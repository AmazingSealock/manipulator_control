#include "arx5_hw/ARX5HWLoop.h"

namespace arx5 {
// Constructor of the ARX5HWLoop class.
// nh: A reference to the ROS node handle.
// hardware_interface: A shared pointer to the ARX5 hardware interface.
ARX5HWLoop::ARX5HWLoop(ros::NodeHandle& nh, std::shared_ptr<ARX5HW> hardware_interface)
    : nh_(nh), hardwareInterface_(std::move(hardware_interface)), loopRunning_(true) {
  // Create the controller manager, which is responsible for managing robot controllers.
  controllerManager_.reset(new controller_manager::ControllerManager(hardwareInterface_.get(), nh_));

  // Load ROS parameters from the parameter server.
  int error = 0;
  int threadPriority = 0;
  // Create a private node handle to access private parameters.
  ros::NodeHandle nhP("~");
  // Retrieve the loop frequency parameter. If retrieval fails, increment the error counter.
  error += static_cast<int>(!nhP.getParam("loop_frequency", loopHz_));
  // Retrieve the cycle time error threshold parameter. If retrieval fails, increment the error counter.
  error += static_cast<int>(!nhP.getParam("cycle_time_error_threshold", cycleTimeErrorThreshold_));
  // Retrieve the thread priority parameter. If retrieval fails, increment the error counter.
  error += static_cast<int>(!nhP.getParam("thread_priority", threadPriority));
  // If any parameter retrieval fails, print an error message and throw an exception.
  if (error > 0) {
    std::string error_message =
        "could not retrieve one of the required parameters: loop_hz or cycle_time_error_threshold or thread_priority";
    ROS_ERROR_STREAM(error_message);
    throw std::runtime_error(error_message);
  }

  // Get the current time, which will be used for the first update.
  lastTime_ = Clock::now();

  // Setup the loop thread. The thread will continuously call the update function while the loop is running.
  loopThread_ = std::thread([&]() {
    while (loopRunning_) {
      update();
    }
  });
  // Set the scheduling parameters for the thread. Here we use the SCHED_FIFO scheduling policy.
  sched_param sched{.sched_priority = threadPriority};
  // Try to set the thread priority. If it fails, print a warning message.
  if (pthread_setschedparam(loopThread_.native_handle(), SCHED_FIFO, &sched) != 0) {
    ROS_WARN(
        "Failed to set threads priority (one possible reason could be that the user and the group permissions "
        "are not set properly.).\n");
  }
}

// The main update function of the loop. It performs the input, control, and output operations in a loop.
void ARX5HWLoop::update() {
  // Get the current time.
  const auto currentTime = Clock::now();
  // Compute the desired duration of each loop cycle based on the loop frequency.
  const Duration desiredDuration(1.0 / loopHz_);

  // Calculate the time elapsed since the last update.
  Duration time_span = std::chrono::duration_cast<Duration>(currentTime - lastTime_);
  elapsedTime_ = ros::Duration(time_span.count());
  // Update the last time to the current time for the next iteration.
  lastTime_ = currentTime;

  // Check if the cycle time exceeds the error threshold.
  const double cycle_time_error = (elapsedTime_ - ros::Duration(desiredDuration.count())).toSec();
  if (cycle_time_error > cycleTimeErrorThreshold_) {
    ROS_WARN_STREAM("Cycle time exceeded error threshold by: " << cycle_time_error - cycleTimeErrorThreshold_ << "s, "
                                                               << "cycle time: " << elapsedTime_ << "s, "
                                                               << "threshold: " << cycleTimeErrorThreshold_ << "s");
  }

  // Input step: Read the current state of the hardware.
  hardwareInterface_->read(ros::Time::now(), elapsedTime_);

  // Control step: Let the controller manager compute the new command based on the current state.
  controllerManager_->update(ros::Time::now(), elapsedTime_);

  // Output step: Send the new command to the hardware.
  hardwareInterface_->write(ros::Time::now(), elapsedTime_);

  // Sleep for the remaining time of the desired cycle duration to maintain the loop frequency.
  const auto sleepTill = currentTime + std::chrono::duration_cast<Clock::duration>(desiredDuration);
  std::this_thread::sleep_until(sleepTill);
}

// Destructor of the ARX5HWLoop class.
// It stops the loop thread and waits for it to finish.
ARX5HWLoop::~ARX5HWLoop() {
  // Set the loop running flag to false to stop the loop thread.
  loopRunning_ = false;
  // If the thread is joinable, wait for it to finish.
  if (loopThread_.joinable()) {
    loopThread_.join();
  }
}

}  // namespace arx5