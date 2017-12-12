#pragma once

#include <cassert>
#include <string>
#include <joystick_interface/joystick_state_interface.h>

namespace hardware_interface
{
	
	// Class to buffer data needed to set the state of the
	// Joystick.  This should (eventually) include anything
	// which might be set during runtime.  Config data
	// which is set only once at startup can be handled
	// in the hardware manager constructor/init rather than through
	// this interface.
	// Various controller code will set the member vars of 
	// this class depending on the needs of the motor 
	// being controlled
	// Each pass through write() in the hardware interface
	// will use this to re-configure (if necessary) and then
	// update the setpoint on the associated Joystick.
	// The hardware_controller is responsible for keeping
	// a master array of these classes - 1 entry per
	// physical Joystick controller in the robot
	class JoystickHWCommand
	{
		public:
			JoystickHWCommand(void) :
				command_(0.0)
				
			{
			}
			// This gets the requested setpoint, not the
			// status actually read from the controller
			// Need to think about which makes the most
			// sense to query...
			bool get(double &command)
			{
				command = command_;
				if (!command_changed_)
					return false;
				command_changed_ = false;
				return true;
			}

			void setRumble(double command, int which) {command_changed_ = true; command_ = command;}
			// Check  to see if mode changed since last call
			// If so, return true and set mode to new desired
			// joystick mode
			// If mode hasn't changed, return false
			// Goal here is to prevent writes to the CAN
			// bus to repeatedly set the mode to the same value. 
			// Instead, only send a setMode to a given Joystick if 
			// the mode has actually changed.
		private:
			double    command_; // motor setpoint - % vbus, velocity, position, etc
			bool      command_changed_;
	};

	// Handle - used by each controller to get, by name of the
	// corresponding joint, an interface with which to send commands
	// to a Joystick
	class JoystickCommandHandle: public JoystickStateHandle
	{
		public:
			JoystickCommandHandle(void) : 
				JoystickStateHandle(), 
				cmd_(0)
			{
			}

			JoystickCommandHandle(const JoystickStateHandle &js, JoystickHWCommand *cmd) : 
				JoystickStateHandle(js), 
				cmd_(cmd)
			{
				if (!cmd_)
					throw HardwareInterfaceException("Cannot create Joystick handle '" + js.getName() + "'. command pointer is null.");
			}

			// Operator which allows access to methods from
			// the JoystickHWCommand member var associated with this
			// handle
			// Note that we could create separate methods in
			// the handle class for every method in the HWState
			// class, e.g.
			//     double getFoo(void) const {assert(_state); return state_->getFoo();}
			// but if each of them just pass things unchanged between
			// the calling code and the HWState method there's no
			// harm in making a single method to do so rather than
			// dozens of getFoo() one-line methods
			// 
			JoystickHWCommand * operator->() {assert(cmd_); return cmd_;}

			// Get a pointer to the HW state associated with
			// this Joystick.  Since CommandHandle is derived
			// from StateHandle, there's a state embedded
			// in each instance of a CommandHandle. Use
			// this method to access it.
			//
			// handle->state()->getCANID();
			//
			const JoystickHWState * state(void) const { return JoystickStateHandle::operator->(); }

		private:
			JoystickHWCommand *cmd_;
	};

	// Use ClaimResources here since we only want 1 controller
	// to be able to access a given Joystick at any particular time
	class JoystickCommandInterface : public HardwareResourceManager<JoystickCommandHandle, ClaimResources> {};
}
