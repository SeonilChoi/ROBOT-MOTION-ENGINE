#ifndef ROBOT_MOTION_ENGINE_HPP_
#define ROBOT_MOTION_ENGINE_HPP_

#include <memory>
#include <vector>
#include <string>

#include "robot_motion_engine/core/robot_controller.hpp"

namespace micros {

class RobotMotionEngine {
public:
    RobotMotionEngine();
    ~RobotMotionEngine() = default;

    void load(const std::string& config_file);

    void home();

    void start();

    void stop();

private:
    std::vector<std::unique_ptr<RobotController>> robot_controllers_;
};

} // namespace micros
#endif // ROBOT_MOTION_ENGINE_HPP_