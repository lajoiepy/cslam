#include <rclcpp/rclcpp.hpp>
//#include <ros/publisher.h>
//#include <ros/subscriber.h>

#include <rtabmap_ros/msg/map_data.hpp>
#include <rtabmap_ros/msg/info.hpp>
#include <rtabmap_ros/MsgConversion.h>
#include <rtabmap_ros/srv/add_link.hpp>
#include <rtabmap_ros/srv/get_map.hpp>

#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/VWDictionary.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/RegistrationVis.h>
#include <rtabmap/utilite/UStl.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <cslam_interfaces/srv/detect_loop_closure.hpp>
#include <cv_bridge/cv_bridge.h>

/*
 * Test:
 * $ roslaunch rtabmap_ros demo_robot_mapping.launch
 * Disable internal loop closure detection, in rtabmapviz->Preferences:
 *    ->Vocabulary, set Max words to -1 (loop closure detection disabled)
 *    ->Proximity Detection, uncheck proximity detection by space
 * $ rosrun rtabmap_ros external_loop_detection_example
 * $ rosbag play --clock demo_mapping.bag
 */

typedef message_filters::sync_policies::ExactTime<rtabmap_ros::msg::MapData, rtabmap_ros::msg::Info> MyInfoMapSyncPolicy;


// Use an external service for loop closure detection
class ExternalLoopClosureService
{
	public:
	ExternalLoopClosureService(){};

	~ExternalLoopClosureService(){};

	void init(std::shared_ptr<rclcpp::Node>& node)
	{
		node_ = node;
		client_ = node_->create_client<cslam_interfaces::srv::DetectLoopClosure>("detect_loop_closure");
	}

	bool process(const rtabmap::SensorData& data, const int id)
	{
		RCLCPP_DEBUG(node_->get_logger(), "Process Image %d for Loop Closure Detection", id);
		// Image message
		std_msgs::msg::Header header;
		//header.seq = id; TODO: add id to message
		header.stamp = node_->now();
		cv_bridge::CvImage image_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, data.imageRaw());
		sensor_msgs::msg::Image image_msg;
		image_bridge.toImageMsg(image_msg);

		// Service request
		auto request = std::make_shared<cslam_interfaces::srv::DetectLoopClosure::Request>();
		request->image = image_msg;

		auto result = client_->async_send_request(request);
		if (rclcpp::spin_until_future_complete(node_, result) ==
    		rclcpp::FutureReturnCode::SUCCESS)
		{
			loop_closure_id_ = result.get()->detected_loop_closure_id;
			RCLCPP_DEBUG(node_->get_logger(), "Loop Closure Detection service success: %d", ((int) result.get()->is_detected));
			return result.get()->is_detected;
		}
		else
		{
			RCLCPP_ERROR(node_->get_logger(), "Failed to call loop closure detection service");
			loop_closure_id_ = 0;
			return false;
		}
	}

	int getLoopClosureId()
	{
		return loop_closure_id_;
	}

	private:
	rclcpp::Client<cslam_interfaces::srv::DetectLoopClosure>::SharedPtr client_;
	int loop_closure_id_ = 0;
	std::shared_ptr<rclcpp::Node> node_;

};

class ExternalLoopClosureDetection
{
	public:
	ExternalLoopClosureDetection(){};
	~ExternalLoopClosureDetection(){};

	void init(std::shared_ptr<rclcpp::Node>& node){
		node_ = node;
		loopClosureDetector_.init(node_);
		
		// service to add link
		addLinkSrv_ = node_->create_client<rtabmap_ros::srv::AddLink>("/rtabmap/add_link");

		// subscription
		message_filters::Subscriber<rtabmap_ros::msg::Info> infoTopic;
		message_filters::Subscriber<rtabmap_ros::msg::MapData> mapDataTopic;
		message_filters::Synchronizer<MyInfoMapSyncPolicy> * infoMapSync;

		infoTopic.subscribe(node_.get(), "/rtabmap/info");
		mapDataTopic.subscribe(node_.get(), "/rtabmap/mapData");
		infoMapSync = new message_filters::Synchronizer<MyInfoMapSyncPolicy>(
				MyInfoMapSyncPolicy(10),
				mapDataTopic,
				infoTopic);
		infoMapSync->registerCallback(&ExternalLoopClosureDetection::mapDataCallback, this);

	}

	void mapDataCallback(const std::shared_ptr<rtabmap_ros::msg::MapData> & mapDataMsg, const std::shared_ptr<rtabmap_ros::msg::Info> & infoMsg)
	{
		RCLCPP_DEBUG(node_->get_logger(), "Received map data!");

		rtabmap::Statistics stats;
		rtabmap_ros::infoFromROS(*infoMsg, stats);

		bool smallMovement = (bool)uValue(stats.data(), rtabmap::Statistics::kMemorySmall_movement(), 0.0f);
		bool fastMovement = (bool)uValue(stats.data(), rtabmap::Statistics::kMemoryFast_movement(), 0.0f);

		if(smallMovement || fastMovement)
		{
			// The signature has been ignored from rtabmap, don't process it
			RCLCPP_DEBUG(node_->get_logger(), "Ignore keyframe. Small movement=%d, Fast movement=%d", (int)smallMovement, (int)fastMovement);
			return;
		}

		rtabmap::Transform mapToOdom;
		std::map<int, rtabmap::Transform> poses;
		std::multimap<int, rtabmap::Link> links;
		std::map<int, rtabmap::Signature> signatures;
		rtabmap_ros::mapDataFromROS(*mapDataMsg, poses, links, signatures, mapToOdom);

		if(!signatures.empty() &&
			signatures.rbegin()->second.sensorData().isValid() &&
			localData_.find(signatures.rbegin()->first) == localData_.end())
		{
			int id = signatures.rbegin()->first;
			const rtabmap::SensorData & s =  signatures.rbegin()->second.sensorData();
			cv::Mat rgb;
			//rtabmap::LaserScan scan;
			s.uncompressDataConst(&rgb, 0/*, &scan*/);
			//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = rtabmap::util3d::laserScanToPointCloud(scan, scan.localTransform());

			if(loopClosureDetector_.process(rgb, id))
			{
				if(loopClosureDetector_.getLoopClosureId()>0)
				{
					int fromId = id;
					int toId = loopClosureDetector_.getLoopClosureId();
					RCLCPP_DEBUG(node_->get_logger(), "Detected loop closure between %d and %d", fromId, toId);
					if(localData_.find(toId) != localData_.end())
					{
						//Compute transformation
						// Registration params
						int min_inliers = 20;
						if (node_->has_parameter("~min_inliers")) {
							node_->get_parameter("~min_inliers", min_inliers);
						}
						rtabmap::ParametersMap params;
						params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kVisMinInliers(), std::to_string(min_inliers)));
						rtabmap::RegistrationVis reg;
						reg.parseParameters(params);
						rtabmap::RegistrationInfo regInfo;
						rtabmap::SensorData tmpFrom = s;
						rtabmap::SensorData tmpTo = localData_.at(toId);
						tmpFrom.uncompressData();
						tmpTo.uncompressData();
						rtabmap::Transform t = reg.computeTransformation(tmpFrom, tmpTo, rtabmap::Transform(), &regInfo);

						if(!t.isNull())
						{
							rtabmap::Link link(fromId, toId, rtabmap::Link::kUserClosure, t, regInfo.covariance.inv());
							auto request = std::make_shared<rtabmap_ros::srv::AddLink::Request>();
							rtabmap_ros::linkToROS(link, request->link);
							auto result = addLinkSrv_->async_send_request(request);
							if(!(rclcpp::spin_until_future_complete(node_, result) ==
    							 rclcpp::FutureReturnCode::SUCCESS))
							{
								RCLCPP_ERROR(node_->get_logger(), "Failed to call service");
							}
						}
						else
						{
							RCLCPP_WARN(node_->get_logger(), "Could not compute transformation between %d and %d: %s", fromId, toId, regInfo.rejectedMsg.c_str());
						}
					}
					else
					{
						RCLCPP_WARN(node_->get_logger(), "Could not compute transformation between %d and %d because node data %d is not in cache.", fromId, toId, toId);
					}
				}
			}

			localData_.insert(std::make_pair(id, s));
		}
	}
	private:

	ExternalLoopClosureService loopClosureDetector_;

	rclcpp::Client<rtabmap_ros::srv::AddLink>::SharedPtr addLinkSrv_;

	std::map<int, rtabmap::SensorData> localData_;

	std::shared_ptr<rclcpp::Node> node_;
	
};

int main(int argc, char** argv)
{

	rclcpp::init(argc, argv);

	auto node = std::make_shared<rclcpp::Node>("external_loop_closure_detection");

	auto lcd = ExternalLoopClosureDetection();
	//lcd.init(node);

	rclcpp::Rate rate(1);

	rclcpp::spin(node);

	rclcpp::shutdown();

	return 0;
}
