#ifndef ROBOT_MANAGER_HPP_
#define ROBOT_MANAGER_HPP_

#include <memory>
#include <vector>
#include <string>

#include "robot_motion_engine/core/robot_controller.hpp"

namespace micros {

class RobotManager {
public:
    RobotManager();
    ~RobotManager() = default;

    void load(const std::string& config_file);

    void start();

    void stop();

private:
    std::vector<std::unique_ptr<RobotController>> robot_controllers_;
};

} // namespace micros
#endif // ROBOT_MANAGER_HPP_