#pragma once

#include <controller_interface/controller.h>
#include <controller_interface/multi_interface_controller.h>
#include <ros/node_handle.h>
#include <std_msgs/Float64.h>
#include <realtime_tools/realtime_buffer.h>
#include <joystick_controllers/joystick_controller_interface.h>

namespace joystick_controllers
{

/**
 * \brief Simple Joystick Controllers`
 *
 * These classes implement simple controllers for JoystickSRX
 * hardware running in various modes.
 *
 * \section ROS interface
 *
 * \param type Must be "Joystick<type>Controller".
 * \param joint Name of the joystick-controlled joint to control.
 *
 * Subscribes to:
 * - \b command (std_msgs::Float64) : The joint setpoint to apply
 */


// Since most controllers are going to share a lot of common code,
// create a base class template. The big difference between controllers
// will be the mode the Joystick is run in. This is specificed by the type
// of joystick interface, so make this the templated parameter.
template <class TALON_IF>
class JoystickController: 
public controller_interface::Controller<hardware_interface::JoystickCommandInterface>
{
	public:
		JoystickController() {}
		~JoystickController() {sub_command_.shutdown();}

		bool init(hardware_interface::JoystickCommandInterface* hw, ros::NodeHandle &n)
		{
			// Read params from command line / config file
			if (!joystick_if_.initWithNode(hw, nullptr, n))
				return false;

			// Might wantt to make message type a template
			// parameter as well?
			sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &JoystickController::commandCB, this);
			return true;
		}

		void starting(const ros::Time& /*time*/)
		{
			// Start controller with motor stopped
			// for great safety
			command_buffer_.writeFromNonRT(0.0);
		}
		void update(const ros::Time& /*time*/, const ros::Duration& /*period*/)
		{
			// Take the most recent value stored in the command
			// buffer (the most recent value read from the "command"
			// topic) and set the Joystick to that commanded value
			joystick_if_.setCommand(*command_buffer_.readFromRT());
		}

	private:
		JOYSTICK_IF joystick_if_;
		ros::Subscriber sub_command_;

		// Real-time buffer holds the last command value read from the 
		// "command" topic.  This buffer is read in each call to update()
		// to get the command to send to the Joystick
		realtime_tools::RealtimeBuffer<double> command_buffer_;

		// Take each message read from the "command" topic and push
		// it into the command buffer. This buffer will later be
		// read by update() and sent to the Joystick.  The buffer
		// is used because incoming messages aren't necessarily
		// synchronized to update() calls - the buffer holds the
		// most recent command value that update() can use
		// when the update() code is run.
		void commandCB(const std_msgs::Float64ConstPtr& msg)
		{
			command_buffer_.writeFromNonRT(msg->data);
		}
};


} // end namespace
