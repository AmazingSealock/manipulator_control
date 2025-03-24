#include "arx5_hw/ARX5HW.h"

#include "arx5_hw/ARX5HWLoop.h"

/**
 * @brief The main function of the ARX5 hardware interface node.
 * 
 * This function initializes the ROS node, sets up the hardware interface for the ARX5 robot,
 * starts the control loop, and waits for a shutdown signal.
 * 
 * @param argc The number of command-line arguments.
 * @param argv The array of command-line arguments.
 * @return 0 if the program exits successfully, 1 if there is an error in the hardware interface.
 */
int main(int argc, char** argv) {
    // Initialize the ROS node with the name "arx5_hw".
    ros::init(argc, argv, "arx5_hw");
    // Create a global ROS node handle.
    ros::NodeHandle nh;
    // Create a private ROS node handle for the hardware interface.
    ros::NodeHandle robotHwNh("~");

    // Run the hardware interface node
    // -------------------------------

    // We run the ROS loop in a separate thread as external calls, such
    // as service callbacks loading controllers, can block the (main) control loop

    // Create an asynchronous spinner with 3 threads to handle ROS callbacks.
    ros::AsyncSpinner spinner(3);
    // Start the asynchronous spinner.
    spinner.start();

    try {
        // Create the hardware interface specific to your robot
        // Allocate memory for the ARX5 hardware interface using a shared pointer.
        std::shared_ptr<arx5::ARX5HW> ARX5HW = std::make_shared<arx5::ARX5HW>();
        // Initialize the hardware interface:
        // 1. retrieve configuration from rosparam
        // 2. initialize the hardware and interface it with ros_control
        // Call the init function of the ARX5 hardware interface to set it up.
        ARX5HW->init(nh, robotHwNh);

        // Start the control loop
        // Create an instance of the control loop and pass the node handle and hardware interface to it.
        arx5::ARX5HWLoop controlLoop(nh, ARX5HW);

        // Wait until shutdown signal received
        // Block the main thread until a shutdown signal is received.
        ros::waitForShutdown();
    } catch (const ros::Exception& e) {
        // If an exception occurs, log a fatal error message with the exception details.
        ROS_FATAL_STREAM("Error in the hardware interface:\n"
                         << "\t" << e.what());
        return 1;
    }

    return 0;
}