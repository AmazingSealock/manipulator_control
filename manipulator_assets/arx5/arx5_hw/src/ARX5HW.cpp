#include "arx5_hw/ARX5HW.h"

namespace arx5 {
/**
 * @brief Initialize the ARX5 hardware interface.
 * 
 * This function initializes the ARX5 robot, loads the URDF model, registers the hardware interfaces,
 * and sets up the joints.
 * 
 * @param root_nh The root ROS node handle.
 * @param robot_hw_nh The robot hardware-specific ROS node handle.
 * @return true if the initialization is successful, false otherwise.
 */
bool ARX5HW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
    // Initialize the ARX5 robot with the specified control mode
    ARX5 = std::make_unique<arx_arm>(CONTROL_MODE);

    // Load the URDF model. If the loading fails, log an error and return false.
    if (!loadUrdf(root_nh)) {
        ROS_ERROR("Error occurred while setting up urdf");
        return false;
    }

    // Register the joint state interface, which is used to read joint states.
    registerInterface(&jointStateInterface_);
    // Register the effort joint interface, which is used to control joint efforts.
    registerInterface(&effortJointInterface_);
    // Register the position joint interface, which is used to control joint positions.
    registerInterface(&positionJointInterface_);
    // Currently commented out. This would register the robot state interface.
    // registerInterface(&robotStateInterface_);

    // Set up the joints for the hardware interface.
    setupJoints();
    return true;
}

/**
 * @brief Load the URDF model from the ROS parameter server.
 * 
 * This function retrieves the URDF string from the parameter server and initializes the URDF model.
 * 
 * @param rootNh The root ROS node handle.
 * @return true if the URDF model is successfully loaded, false otherwise.
 */
bool ARX5HW::loadUrdf(ros::NodeHandle& rootNh) {
    std::string urdfString;
    // If the URDF model pointer is null, create a new URDF model object.
    if (urdfModel_ == nullptr) {
        urdfModel_ = std::make_shared<urdf::Model>();
    }
    // Retrieve the URDF string from the parameter server.
    rootNh.getParam("robot_description", urdfString);
    // Check if the URDF string is not empty and if the URDF model can be initialized with it.
    return !urdfString.empty() && urdfModel_->initString(urdfString);
}

/**
 * @brief Read the joint states from the hardware.
 * 
 * This function reads the joint positions, velocities, and efforts from the ARX5 robot and stores them
 * in the joint data structure. It also calls a CAN handling function.
 * 
 * @param time The current ROS time.
 * @param period The time duration since the last read operation.
 */
void ARX5HW::read(const ros::Time& time, const ros::Duration& period) {
    // Get the current joint states from the ARX5 robot.
    ARX5->get_joint();
    // Iterate over all joints and update the joint data with the actual values.
    for (int i = 0; i < static_cast<int>(jointName.size()); ++i)
    {
        jointData_[i].pos_ = rv_motor_msg[jointID[i]].angle_actual_rad;
        jointData_[i].vel_ = rv_motor_msg[jointID[i]].speed_actual_rad;
        jointData_[i].tau_ = rv_motor_msg[jointID[i]].current_actual_float;
    }
    // Call the CAN handling function to process CAN messages.
    CAN_Handlej.arx_1();
}

/**
 * @brief Write the joint commands to the hardware.
 * 
 * This function sends the joint position commands to the ARX5 robot. If the robot is not starting,
 * it gets the current command from the robot.
 * 
 * @param time The current ROS time (not used in this function).
 * @param period The time duration since the last write operation (not used in this function).
 */
void ARX5HW::write(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
    command cmd;
    // If the ARX5 robot is not starting, get the current command.
    if(!ARX5->is_starting){
        cmd = ARX5->get_cmd();
    }
    // Update the real - time state of the ARX5 robot with the command.
    ARX5->update_real(cmd);
    // Iterate over all joints and set the position commands for the ARX5 robot.
    for (int i = 0; i < static_cast<int>(jointName.size()); ++i)
    {
        // Currently commented out. This would set the effort command for the joint.
        // ARX5->ros_control_cur[i] = jointData_[i].cmdTau_;
        ARX5->ros_control_pos[i] = jointData_[i].cmdPos_;
    }
}

/**
 * @brief Set up the joints for the hardware interface.
 * 
 * This function registers the joint state and position handles for each joint in the hardware interface.
 * 
 * @return true if the joints are successfully set up, false otherwise.
 */
bool ARX5HW::setupJoints() {
    // Iterate over all joints and register the joint state and position handles.
    for (int i = 0; i < static_cast<int>(jointName.size()); ++i)
    {
        // Create a joint state handle for the current joint.
        hardware_interface::JointStateHandle state_handle(jointName[i], &jointData_[i].pos_, &jointData_[i].vel_,
                                                          &jointData_[i].tau_);
        // Register the joint state handle with the joint state interface.
        jointStateInterface_.registerHandle(state_handle);
        // Create a joint handle for position control and register it with the position joint interface.
        positionJointInterface_.registerHandle(hardware_interface::JointHandle(state_handle, &jointData_[i].cmdPos_));
        // Currently commented out. This would register the joint handle for effort control.
        // effortJointInterface_.registerHandle(hardware_interface::JointHandle(state_handle, &jointData_[i].cmdTau_));
    }
    return true;
}

}  // namespace arx5