#ifndef CORE_ROBOT_CONTROLLER_HPP_
#define CORE_ROBOT_CONTROLLER_HPP_

#include <vector>
#include <cstdint>

#include "robot_motion_engine/core/planner.hpp"
#include "robot_motion_engine/core/controller.hpp"
#include "robot_motion_engine/core/scheduler.hpp"
#include "robot_motion_engine/types.hpp"

namespace micros {

class RobotController {
public:
    RobotController(const robot_config_t& config)
    : id_(config.id)
    , driver_ids_(config.driver_ids)
    , home_position_(config.home_position)
    , base_configuration_(config.base_configuration)
    , home_configuration_(config.home_configuration)
    , target_configuration_(config.target_configuration)
    , screw_axes_(config.screw_axes)
    , self_collision_(config.self_collision)
    , obstacle_ids_(config.obstacle_ids)
    {}
    virtual ~RobotController() = default;

    virtual void home() = 0;

    virtual void start() = 0;

    virtual void stop() = 0;

protected:
    virtual bool is_collision() = 0;

    Planner* planner_;

    Controller* controller_;

    Scheduler* scheduler_;

    const uint8_t id_;

    std::vector<uint8_t> driver_ids_;

    std::vector<double> home_position_;

    const pose_t base_configuration_;

    const pose_t home_configuration_;

    std::vector<pose_t> target_configuration_;

    std::vector<screw_axis_t> screw_axes_;

    std::vector<dh_parameter_t> self_collision_;

    std::vector<uint8_t> obstacle_ids_;
};

} // namespace micros
#endif // CORE_ROBOT_CONTROLLER_HPP_