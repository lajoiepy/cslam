#include "cslam/back_end/utils/simulated_rendezvous.h"

using namespace cslam;

SimulatedRendezVous::SimulatedRendezVous(std::shared_ptr<rclcpp::Node> &node, 
                                        const std::string& schedule_file, 
                                        const unsigned int &robot_id): node_(node), robot_id_(robot_id), enabled_(true)
{
    try{
        double initial_time = node_->now().seconds();
        std::ifstream schedule(schedule_file);
        if (schedule.is_open())
        {
            std::string line;
            std::getline(schedule, line);
            auto delim0 = line.find(",");
            if (robot_id_ == std::stoul(line.substr(0, delim0)))
            {
                while (delim0 != std::string::npos){
                    auto delim1 = line.find(",", delim0 + 1);

                    auto start = std::stod(line.substr(delim0 + 1, delim1)) + initial_time;

                    delim0 = delim1;
                    delim1 = line.find(",", delim0);

                    auto end = std::stod(line.substr(delim0 + 1, delim1)) + initial_time;

                    rendezvous_ranges_.push_back(std::make_pair(start, end));

                    delim0 = delim1;
                }
            }
            schedule.close();
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node_->get_logger(), "Reading simulated rendezvous schedule failed: %s", e.what());
        enabled_ = false;
    }
}

bool SimulatedRendezVous::is_alive()
{
    if (enabled_) {
        bool is_alive = false;
        double current_time = node_->now().seconds();

        for (const auto& range: rendezvous_ranges_)
        {
            if (current_time > range.first && current_time < range.second)
            {
                is_alive = true;
            }
        }
        return is_alive;
    }
    else {
        return true;
    }
}