#ifndef _CSLAM_SIMULATED_RENDEZVOUS_H_
#define _CSLAM_SIMULATED_RENDEZVOUS_H_

#include <vector>
#include <string>
#include <iostream>
#include <fstream>

#include <rclcpp/rclcpp.hpp>

#include <ctime>
#include <chrono>
#include <iomanip>

namespace cslam
{

    class SimulatedRendezVous
    { // TODO: document
    public:
        SimulatedRendezVous(std::shared_ptr<rclcpp::Node> &node, const std::string& schedule_file, const unsigned int &robot_id);

        bool is_alive();

    private:
        std::shared_ptr<rclcpp::Node> node_;
        unsigned int robot_id_;
        std::vector<std::pair<uint64_t, uint64_t>> rendezvous_ranges_;
        bool enabled_;
    };

} // namespace cslam

#endif // _CSLAM_SIMULATED_RENDEZVOUS_H_