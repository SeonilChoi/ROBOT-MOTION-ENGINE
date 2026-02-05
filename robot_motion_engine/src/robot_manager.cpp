#include <stdexcept>

#include <yaml-cpp/yaml.h>

#include "robot_motion_engine/robot_manager.hpp"
#include "robot_motion_engine/types.hpp"

micros::RobotManager::RobotManager(const std::string& config_file) 
{
    load(config_file);
}

void micros::RobotManager::start()
{
    for (auto& robot_controller : robot_controllers_) {
        robot_controller->start();
    }
}

void micros::RobotManager::stop()
{
    for (auto& robot_controller : robot_controllers_) {
        robot_controller->stop();
    }
}

void micros::RobotManager::load(const std::string& config_file)
{
    YAML::Node config = YAML::LoadFile(config_file);
    if (!config["robots"] || !config["robots"].IsSequence()) {
        throw std::runtime_error("Invalid configuration file.");
    }

    YAML::Node robots = config["robots"];
    for (auto robot : robots) {
        robot_config_t r_cfg{};
        r_cfg.id = robot["id"].as<uint8_t>();
        r_cfg.driver_ids = robot["driver_ids"].as<std::vector<uint8_t>>();
        r_cfg.home_position = robot["home_position"].as<std::vector<double>>();
        r_cfg.base_configuration.position = robot["base_configuration"]["position"].as<std::array<double, 3>>();
        r_cfg.base_configuration.orientation = robot["base_configuration"]["orientation"].as<std::array<double, 3>>();
        r_cfg.home_configuration.position = robot["home_configuration"]["position"].as<std::array<double, 3>>();
        r_cfg.home_configuration.orientation = robot["home_configuration"]["orientation"].as<std::array<double, 3>>();
        for (auto target : robot["target_configuration"]) {
            r_cfg.target_configuration.push_back({
                target["position"].as<std::array<double, 3>>(),
                target["orientation"].as<std::array<double, 3>>()
            });
        }
        for (auto screw_axis : robot["screw_axes"]) {
            r_cfg.screw_axes.push_back({
                screw_axis["w"].as<std::array<double, 3>>(),
                screw_axis["v"].as<std::array<double, 3>>()
            });
        }
        for (auto self_collision : robot["self_collision"]) {
            r_cfg.self_collision.push_back({
                self_collision["a"].as<double>(),
                self_collision["alpha"].as<double>(),
                self_collision["d"].as<double>(),
                self_collision["theta"].as<double>()
            });
        }
        r_cfg.obstacle_ids = robot["obstacle_ids"].as<std::vector<uint8_t>>();

        switch (convert_robot_type(robot["type"].as<std::string>())) {
        case robot_type_t::OPEN_CHAIN:
            robot_controllers_.push_back(std::make_unique<OpenChainRobotController>(r_cfg));
            break;
        default:
            throw std::runtime_error("Invalid robot type.");
        }
    }


}