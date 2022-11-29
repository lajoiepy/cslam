#include "cslam/back_end/utils/simulated_rendezvous.h"

using namespace cslam;

SimulatedRendezVous::SimulatedRendezVous(std::shared_ptr<rclcpp::Node> &node,
                                         const std::string &schedule_file,
                                         const unsigned int &robot_id) : node_(node), robot_id_(robot_id), enabled_(true)
{
    try
    {
        int64_t initial_time = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        std::ifstream schedule(schedule_file);
        if (schedule.is_open())
        {
            for (std::string line; std::getline(schedule, line);)
            {
                auto delim0 = line.find(",");
                if (robot_id_ == std::stoul(line.substr(0, delim0)))
                {
                    RCLCPP_INFO(node_->get_logger(), "Simulated rendezvous schedule of robot " + line);
                    while (delim0 != std::string::npos)
                    {
                        auto delim1 = line.find(",", delim0 + 1);
                        
                        auto start = std::stoull(line.substr(delim0 + 1, delim1)) + initial_time;

                        delim0 = delim1;
                        delim1 = line.find(",", delim0);

                        auto end = std::stoull(line.substr(delim0 + 1, delim1)) + initial_time;

                        rendezvous_ranges_.push_back(std::make_pair(start, end));

                        delim0 = line.find(",", delim1 +1);
                    }
                }
            }
            schedule.close();
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Unable to open rendezvous schedule file");
            enabled_ = false;
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
    if (enabled_)
    {
        bool is_alive = false;
        uint64_t current_time = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();

        for (const auto &range : rendezvous_ranges_)
        {
            //RCLCPP_INFO(node_->get_logger(), "Time until rendezvous (%d,%d) = %d | Time until rendezvous end = %d", ((int) range.first), ((int) range.second), range.first - ((int)current_time), range.second - ((int)current_time));
            if (current_time >= range.first && current_time <= range.second)
            {
                is_alive = true;
            }
        }
        return is_alive;
    }
    return true;
}