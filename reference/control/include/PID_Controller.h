#define POS_CONTROLLER_PD_H
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <math.h>
namespace PID
{
    class pos_controller_PID
    {
        // public表明该数据成员、成员函数是对全部用户开放的。全部用户都能够直接进行调用，在程序的不论什么其他地方訪问。
    public:
        // 构造函数
        pos_controller_PID(void) : //~:这是节点句柄的命名空间。波浪符号 (~) 用于表示当前节点的私有命名空间。
                                   pos_pid_nh("~")
        {
            // 位置环
            // pos-xyz
            pos_drone = Eigen::Vector3d(0.0, 0.0, 0.0);
            // Desired position and posocity of the drone
            // 期望的无人机位置和速度
            pos_setpoint = Eigen::Vector3d(0.0, 0.0, 0.0);
            // 推力？
            // PID parameter
            pos_P_output = Eigen::Vector3d(0.0, 0.0, 0.0);
            pos_D_output = Eigen::Vector3d(0.0, 0.0, 0.0);
            pos_I_output = Eigen::Vector3d(0.0, 0.0, 0.0);
            //
            error_pos_dot_last = Eigen::Vector3d(0.0, 0.0, 0.0);
            error_pos_last = Eigen::Vector3d(0.0, 0.0, 0.0);
            error_pos_dot_now = Eigen::Vector3d(0.0, 0.0, 0.0);
            delta_time = 0.02;
        }
        float MPC_XY_pos_P = 1.05;
        float MPC_Z_pos_P = 0.9;

        float MPC_XY_pos_D = 0.15;
        float MPC_Z_pos_D = 0.15;

        float MPC_XY_pos_I = 0.001;
        float MPC_Z_pos_I = 0.001;

        Eigen::Vector3d pos_drone;
        Eigen::Vector3d pos_setpoint;
        Eigen::Vector3d pos_P_output;
        Eigen::Vector3d pos_D_output;
        Eigen::Vector3d pos_I_output;
        Eigen::Vector3d error_pos_dot_last;
        Eigen::Vector3d error_pos_last;
        Eigen::Vector3d error_pos_dot_now;
        float delta_time;
        // posocity control loop [Input: current pos, desired pos; Output: desired thrust]
        Eigen::Vector3d posController(Eigen::Vector3d error_pos, Eigen::Vector3d drone_vel, double delta_time);
        Eigen::Vector3d cal_pos_error_deriv(Eigen::Vector3d error_now);

    private:
        ros::NodeHandle pos_pid_nh;
    };
    Eigen::Vector3d pos_controller_PID::posController(Eigen::Vector3d error_pos, Eigen::Vector3d drone_vel, double delta_time)
    {
        // delta_time = dt;
        pos_P_output(0) = MPC_XY_pos_P * error_pos(0);
        pos_P_output(1) = MPC_XY_pos_P * error_pos(1);
        pos_P_output(2) = MPC_Z_pos_P * error_pos(2);

        // Eigen::Vector3d pos_error_deriv = cal_pos_error_deriv(error_pos);

        pos_D_output(0) = -MPC_XY_pos_D * drone_vel(0);
        pos_D_output(1) = -MPC_XY_pos_D * drone_vel(1);
        pos_D_output(2) = -MPC_Z_pos_D * drone_vel(2);

        if (error_pos.norm() > 0.4)
        {
            pos_I_output(0) = 0;
            pos_I_output(1) = 0;
            pos_I_output(2) = 0;
        }
        else
        {
            pos_I_output(0) = MPC_XY_pos_I * error_pos(0) * delta_time;
            pos_I_output(1) = MPC_XY_pos_I * error_pos(1) * delta_time;
            pos_I_output(2) = MPC_Z_pos_I * error_pos(2) * delta_time;
            for (size_t i = 0; i < 3; i++)
            {
                if (abs(pos_I_output(i)) > 0.2)
                {
                    pos_I_output(i) *= 0.2 / abs(pos_I_output(i));
                }
            }
        }
        // std::cout << "pos_D_output: " << pos_D_output << std::endl;
        return (pos_P_output + pos_D_output + pos_I_output);
    }

    Eigen::Vector3d pos_controller_PID::cal_pos_error_deriv(Eigen::Vector3d error_now)
    {
        std::cout << "error_now - error_pos_last: " << error_now - error_pos_last << std::endl;
        error_pos_dot_now = (error_now - error_pos_last) / delta_time;
        std::cout << "delta_time: " << delta_time << std::endl;
        // for (int i = 0; i < error_pos_dot_now.size(); ++i)
        // {
        //     if (std::isnan(error_pos_dot_now(i)))
        //     {
        //         error_pos_dot_now(i) = 0;
        //     }
        // }
        // std::cout << "error_pos_dot_now: " << error_pos_dot_now << std::endl;
        error_pos_last = error_now;
        // std::cout << "error_pos_dot_last: " << 0.3 * error_pos_dot_last << std::endl;
        // std::cout << "error_pos_dot_now: " << 0.7 * error_pos_dot_now << std::endl;
        // std::cout << "output: " << 0.7 * error_pos_dot_now + 0.3 * error_pos_dot_last << std::endl;
        error_pos_dot_last = 0.7 * error_pos_dot_now + 0.3 * error_pos_dot_last;
        std::cout << "output: " << error_pos_dot_last << std::endl;

        return error_pos_dot_last;
    }
}