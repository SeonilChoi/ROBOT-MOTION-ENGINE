#ifndef CORE_ROBOT_CONTROLLER_HPP_
#define CORE_ROBOT_CONTROLLER_HPP_

#include "robot_motion_engine/core/planner.hpp"
#include "robot_motion_engine/core/controller.hpp"
#include "robot_motion_engine/core/scheduler.hpp"

namespace micros {

class RobotController {
public:
    RobotController();
    virtual ~RobotController() = default;

    virtual void load() = 0;

    virtual void home() = 0;

    virtual void start() = 0;

    virtual void stop() = 0;

protected:
    virtual bool is_collision() = 0;

    Planner* planner_;

    Controller* controller_;

    Scheduler* scheduler_;
};

} // namespace micros
#endif // CORE_ROBOT_CONTROLLER_HPP_