#ifndef CORE_SCHEDULER_HPP_
#define CORE_SCHEDULER_HPP_

namespace micros {

class Scheduler {
public:
    Scheduler();
    virtual ~Scheduler() = default;

    virtual void reset() = 0;

    virtual void tick() = 0;
};

} // namespace micros
#endif // CORE_SCHEDULER_HPP_