#include "grc26/controller_config.hpp"

ControllerConfig::ControllerConfig()
    : controllers_{
        std::array<PID, 3>{
            PID(1.0, 0.0, 0.0),
            PID(1.0, 0.0, 0.0),
            PID(1.0, 0.0, 0.0)
        },
        std::array<StiffnessController, 3>{
            StiffnessController(1.0),
            StiffnessController(1.0),
            StiffnessController(1.0)
        }
      }
{
    for (int i = 0; i < 3; ++i)
    {
        controllers_.pid_lin[i] = PID(1.0, 0.0, 0.0);
        controllers_.ori_ctrl[i] = StiffnessController(1.0);
    }
}

bool ControllerConfig::load(const std::string& file)
{
    try
    {
        const std::string path =
            ament_index_cpp::get_package_share_directory("grc26") +
            "/config/" + file;

        YAML::Node root = YAML::LoadFile(path);

        auto pid_config = root["linear_vel_pid"];
        if (pid_config)
        {
            for (int i = 0; i < 3; ++i)
            {
                const std::string axis = (i == 0 ? "x" : (i == 1 ? "y" : "z"));
                const YAML::Node node = pid_config[axis];
                if (node)
                {
                    controllers_.pid_lin[i].set_params(
                        node["kp"] ? node["kp"].as<double>() : controllers_.pid_lin[i].kp,
                        node["ki"] ? node["ki"].as<double>() : controllers_.pid_lin[i].ki,
                        node["kd"] ? node["kd"].as<double>() : controllers_.pid_lin[i].kd,
                        node["error_sum_tol"] ? node["error_sum_tol"].as<double>()       : controllers_.pid_lin[i].err_sum_tol,
                        node["decay_rate"] ? node["decay_rate"].as<double>()             : controllers_.pid_lin[i].decay_rate,
                        node["dead_zone_limit"] ? node["dead_zone_limit"].as<double>()   : controllers_.pid_lin[i].dead_zone_limit,
                        node["lp_filter_alpha"] ? node["lp_filter_alpha"].as<double>()   : controllers_.pid_lin[i].lp_filter_alpha,
                        node["saturation_limit"] ? node["saturation_limit"].as<double>() : controllers_.pid_lin[i].saturation_limit);
                }
            }
        }

        auto ori_config = root["orientation_stiffness"];
        if (ori_config)
        {
            for (int i = 0; i < 3; ++i)
            {
                const std::string axis = (i == 0 ? "roll" : (i == 1 ? "pitch" : "yaw"));
                const YAML::Node node = ori_config[axis];
                if (node)
                {
                    controllers_.ori_ctrl[i].set_params(node.as<double>());
                }
            }
        }
        return true;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Failed to load controller config: "
                  << e.what() << std::endl;
        return false;
    }
}