#ifndef ROBOT_MOTION_ENGINE_TYPE_HPP_
#define ROBOT_MOTION_ENGINE_TYPE_HPP_

#include <array>
#include <string>
#include <vector>
#include <cstdint>

namespace micros {

enum class robot_type_t {
    OPEN_CHAIN,
    MOBILE,
};

struct pose_t {
    std::array<double, 3> position;
    std::array<double, 3> orientation;
};

struct screw_axis_t {
    std::array<double, 3> w;
    std::array<double, 3> v;
};

struct dh_parameter_t {
    double a;
    double alpha;
    double d;
    double theta;
};
    
struct robot_config_t {
    uint8_t id;
    std::vector<uint8_t> driver_ids;
    std::vector<double> home_position;
    pose_t base_configuration;
    pose_t home_configuration;
    std::vector<pose_t> target_configuration;
    std::vector<screw_axis_t> screw_axes;
    std::vector<dh_parameter_t> self_collision;
    std::vector<uint8_t> obstacle_ids;
};

inline robot_type_t convert_robot_type(const std::string& type)
{
    if (type == "open_chain") return robot_type_t::OPEN_CHAIN;
    if (type == "mobile") return robot_type_t::MOBILE;
    throw std::runtime_error("Invalid robot type: " + type);
    return robot_type_t::OPEN_CHAIN;
}

} // namespace micros
#endif // ROBOT_MOTION_ENGINE_TYPE_HPP_