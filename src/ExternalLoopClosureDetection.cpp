#include <rclcpp/rclcpp.hpp>
//#include <ros/publisher.h>
//#include <ros/subscriber.h>

#include <rtabmap_ros/msg/map_data.hpp>
#include <rtabmap_ros/msg/info.hpp>
#include <rtabmap_ros/srv/add_link.hpp>
#include <rtabmap_ros/srv/get_map.hpp>
#include <external_loop_closure_detection/MsgConversion.h>

#include <rtabmap/core/SensorData.h>
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

#include <cv_bridge/cv_bridge.h>

#include <cslam_loop_detection/srv/detect_loop_closure.hpp>
#include <cslam_utils/msg/image_id.hpp>
#include <thread> 
#include <chrono> 
#include <deque>

// Message filters to sync callbacks
typedef message_filters::sync_policies::ExactTime<rtabmap_ros::msg::MapData, rtabmap_ros::msg::Info> MyInfoMapSyncPolicy;

// Response
struct LoopClosureResponse
{
	bool is_valid;
	bool is_detected;
	int from_id;
	int to_id;
};

// Use an external service for loop closure detection
class ExternalLoopClosureService
{
	public:
	ExternalLoopClosureService(){};

	~ExternalLoopClosureService(){};

	void init(std::shared_ptr<rclcpp::Node>& node)
	{
		node_ = node;
		client_ = node_->create_client<cslam_loop_detection::srv::DetectLoopClosure>("detect_loop_closure");
		while (!client_->wait_for_service(std::chrono::seconds(1))) {
			if (!rclcpp::ok()) {
				RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
				return;
			}
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
		}
	}

	void detectLoopClosures(const rtabmap::SensorData& data, const int id)
	{
		RCLCPP_INFO(node_->get_logger(), "Process Image %d for Loop Closure Detection", id);
		// Image message
		std_msgs::msg::Header header;
		header.stamp = node_->now();
		cv_bridge::CvImage image_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, data.imageRaw());
		cslam_utils::msg::ImageId image_msg;
		image_bridge.toImageMsg(image_msg.image);
		image_msg.id = id;

		// Service request
		auto request = std::make_shared<cslam_loop_detection::srv::DetectLoopClosure::Request>();
		request->image = image_msg;

		responses_.push_back(client_->async_send_request(request));
		RCLCPP_INFO(node_->get_logger(), "Service called for place recognition processing");
	}

	LoopClosureResponse checkForResponse(){
		LoopClosureResponse res = {false, false, -1, -1};
		if (!responses_.empty())
		{
			auto response = responses_.front();
			auto status = response.wait_for(std::chrono::seconds(0));
			if (status == std::future_status::ready || status == std::future_status::deferred)
			{	
				responses_.pop_front();
				int test = response.get()->from_id;
				res = {true, response.get()->is_detected, response.get()->from_id,response.get()->detected_loop_closure_id};
				RCLCPP_INFO(node_->get_logger(), "Image %d processed.", res.from_id);
			}
		}
		return res;
	}

	private:

	rclcpp::Client<cslam_loop_detection::srv::DetectLoopClosure>::SharedPtr client_;
	std::deque<rclcpp::Client<cslam_loop_detection::srv::DetectLoopClosure>::SharedFuture> responses_;
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
		std::string AddLinkSrv;
		node_->get_parameter("add_link_srv", AddLinkSrv);
		addLinkSrv_ = node_->create_client<rtabmap_ros::srv::AddLink>(AddLinkSrv);
		while (!addLinkSrv_->wait_for_service(std::chrono::seconds(1))) {
			if (!rclcpp::ok()) {
				RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
				return;
			}
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
		}

		node_->get_parameter("max_queue_size", maxQueueSize_);
		node_->get_parameter("min_inliers", minInliers_);

		RCLCPP_INFO(node_->get_logger(), "Initialization done.");
	}

	void processNewKeyFrames(){
		if (!receivedDataQueue_.empty())
		{
			auto mapData = receivedDataQueue_.front();
			receivedDataQueue_.pop_front();

			rtabmap::Transform mapToOdom;
			std::map<int, rtabmap::Transform> poses;
			std::multimap<int, rtabmap::Link> links;
			std::map<int, rtabmap::Signature> signatures;
			rtabmap_ros::mapDataFromROS(*mapData, poses, links, signatures, mapToOdom);

			if(!signatures.empty() &&
				signatures.rbegin()->second.sensorData().isValid() &&
				localData_.find(signatures.rbegin()->first) == localData_.end())
			{
				int id = signatures.rbegin()->first;
				const rtabmap::SensorData & s =  signatures.rbegin()->second.sensorData();
				cv::Mat rgb;
				//rtabmap::LaserScan scan;
				s.uncompressDataConst(&rgb, 0);
				//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = rtabmap::util3d::laserScanToPointCloud(scan, scan.localTransform());
				// Send request for loop detection
				loopClosureDetector_.detectLoopClosures(rgb, id);
				
				localData_.insert(std::make_pair(id, s));
			}
		}
	}

	void processLoopClosure(){
		LoopClosureResponse res = loopClosureDetector_.checkForResponse();
		if (res.is_valid)
		{
			if(res.is_detected)
			{
				int fromId = res.from_id;
				int toId = res.to_id;
				RCLCPP_INFO(node_->get_logger(), "Detected loop closure between %d and %d", fromId, toId);
				if(localData_.find(toId) != localData_.end())
				{
					//Compute transformation
					// Registration params
					rtabmap::ParametersMap params;
					params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kVisMinInliers(), std::to_string(minInliers_)));
					rtabmap::RegistrationVis reg;
					reg.parseParameters(params);
					rtabmap::RegistrationInfo regInfo;
					rtabmap::SensorData tmpFrom = localData_.at(fromId);
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
						// Do not need to wait for response
						// if(!(rclcpp::spin_until_future_complete(node_, result) ==
						// 	rclcpp::FutureReturnCode::SUCCESS))
						// {
						// 	RCLCPP_ERROR(node_->get_logger(), "Failed to call service");
						// }
						RCLCPP_INFO(node_->get_logger(), "Add link service called");
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
	}

	void mapDataCallback(const std::shared_ptr<rtabmap_ros::msg::MapData> & mapDataMsg, const std::shared_ptr<rtabmap_ros::msg::Info> & infoMsg)
	{
		RCLCPP_INFO(node_->get_logger(), "Received map data!");

		rtabmap::Statistics stats;
		rtabmap_ros::infoFromROS(*infoMsg, stats);

		bool smallMovement = (bool)uValue(stats.data(), rtabmap::Statistics::kMemorySmall_movement(), 0.0f);
		bool fastMovement = (bool)uValue(stats.data(), rtabmap::Statistics::kMemoryFast_movement(), 0.0f);

		if(smallMovement || fastMovement)
		{
			// The signature has been ignored from rtabmap, don't process it
			RCLCPP_INFO(node_->get_logger(), "Ignore keyframe. Small movement=%d, Fast movement=%d", (int)smallMovement, (int)fastMovement);
			return;
		}

		receivedDataQueue_.push_back(mapDataMsg);
		if (receivedDataQueue_.size() > maxQueueSize_)
		{
			// Remove the oldest keyframes if we exceed the maximum size
			receivedDataQueue_.pop_front();
			RCLCPP_WARN(node_->get_logger(), "Maximum queue size (%d) exceeded, the oldest element was removed.", maxQueueSize_);
		}
	}


	private:

	ExternalLoopClosureService loopClosureDetector_;

	rclcpp::Client<rtabmap_ros::srv::AddLink>::SharedPtr addLinkSrv_;

	std::map<int, rtabmap::SensorData> localData_;

	std::shared_ptr<rclcpp::Node> node_;

	std::deque<std::shared_ptr<rtabmap_ros::msg::MapData>> receivedDataQueue_;

	int maxQueueSize_;

	int minInliers_;
	
};

int main(int argc, char** argv)
{

	rclcpp::init(argc, argv);

	auto node = std::make_shared<rclcpp::Node>("external_loop_closure_detection");

	node->declare_parameter<std::string>("add_link_srv", "/rtabmap/add_link");
	node->declare_parameter<std::string>("rtabmap_info_topic", "/rtabmap/info");
	node->declare_parameter<std::string>("rtabmap_map_topic", "/rtabmap/mapData");
	node->declare_parameter<int>("min_inliers", 20);
	node->declare_parameter<int>("max_queue_size", 10);

	message_filters::Subscriber<rtabmap_ros::msg::Info> infoTopic_;

	message_filters::Subscriber<rtabmap_ros::msg::MapData> mapDataTopic_;

	message_filters::Synchronizer<MyInfoMapSyncPolicy> * infoMapSync_;

	std::string info_topic;
	node->get_parameter("rtabmap_info_topic", info_topic);
	std::string map_topic;
	node->get_parameter("rtabmap_map_topic", map_topic);
	
	infoTopic_.subscribe(node.get(), info_topic);
	mapDataTopic_.subscribe(node.get(), map_topic);
	infoMapSync_ = new message_filters::Synchronizer<MyInfoMapSyncPolicy>(
			MyInfoMapSyncPolicy(10),
			mapDataTopic_,
			infoTopic_);

	auto lcd = ExternalLoopClosureDetection();
	lcd.init(node);

	infoMapSync_->registerCallback(&ExternalLoopClosureDetection::mapDataCallback, &lcd);

	rclcpp::Rate rate(10);

	while (rclcpp::ok())
  	{
		lcd.processNewKeyFrames();
		lcd.processLoopClosure();
		rclcpp::spin_some(node);
		rate.sleep();
	}

	rclcpp::shutdown();

	return 0;
}
