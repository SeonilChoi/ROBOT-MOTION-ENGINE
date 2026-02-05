#ifndef ROBOT_MANAGER_HPP_
#define ROBOT_MANAGER_HPP_

#include <memory>
#include <vector>
#include <string>

#include "robot_motion_engine/core/robot_controller.hpp"

namespace micros {

class RobotManager {
public:
    RobotManager(const std::string& config_file);
    ~RobotManager() = default;

    void start();

    void stop();

private:
    void load(const std::string& config_file);
    
    std::vector<std::unique_ptr<RobotController>> robot_controllers_;
};

} // namespace micros
#endif // ROBOT_MANAGER_HPP_