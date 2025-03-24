#include <string>
#include <ros/ros.h>
// 引入 MoveIt 的 MoveGroupInterface 头文件，用于与机械臂的运动规划和控制交互
#include <moveit/move_group_interface/move_group_interface.h>
// 引入自定义的消息类型，用于传递位置命令
#include <arm_control/PosCmd.h>

int main(int argc, char **argv)
{
    // 初始化 ROS 节点，节点名称为 "moveit_fk_demo"
    ros::init(argc, argv, "moveit_fk_demo");
    // 创建一个异步的 ROS 回调函数执行器，使用 1 个线程
    ros::AsyncSpinner spinner(1);
    // 启动异步回调函数执行器，开始处理 ROS 消息
    spinner.start();

    // 创建一个 MoveGroupInterface 对象，指定要控制的机械臂组名为 "manipulator"
    moveit::planning_interface::MoveGroupInterface arm("manipulator");

    // 设置机械臂运动到目标位置时的关节角度误差容忍度为 0.001
    arm.setGoalJointTolerance(0.001);

    // 设置机械臂运动时的最大加速度缩放因子为 0.2，即最大加速度为原本的 20%
    arm.setMaxAccelerationScalingFactor(0.2);
    // 设置机械臂运动时的最大速度缩放因子为 0.2，即最大速度为原本的 20%
    arm.setMaxVelocityScalingFactor(0.2);

    // 控制机械臂移动到预定义的名为 "home" 的初始位置
    arm.setNamedTarget("home");
    // 执行运动规划并让机械臂移动到目标位置
    arm.move();
    // 程序暂停 1 秒，等待机械臂完成运动
    sleep(1);

    // 定义一个包含 6 个元素的数组，存储目标关节角度值
    double targetPose[6] = {0.8, 0.8, 0.0, 0.0, 0.0, 0.0};
    // 创建一个大小为 6 的向量，用于存储关节组的位置信息
    std::vector<double> joint_group_positions(6);
    // 将目标关节角度值依次赋值给向量中的对应元素
    joint_group_positions[0] = targetPose[0];
    joint_group_positions[1] = targetPose[1];
    joint_group_positions[2] = targetPose[2];
    joint_group_positions[3] = targetPose[3];
    joint_group_positions[4] = targetPose[4];
    joint_group_positions[5] = targetPose[5];

    // 设置机械臂的目标关节位置为上述向量中存储的值
    arm.setJointValueTarget(joint_group_positions);
    // 执行运动规划并让机械臂移动到目标关节位置
    arm.move();
    // 程序暂停 1 秒，等待机械臂完成运动
    sleep(1);

    // 再次控制机械臂移动到预定义的名为 "home" 的初始位置
    arm.setNamedTarget("home");
    // 执行运动规划并让机械臂移动到目标位置
    arm.move();
    // 程序暂停 1 秒，等待机械臂完成运动
    sleep(1);

    // 关闭 ROS 节点，结束节点的运行
    ros::shutdown(); 

    return 0;
}