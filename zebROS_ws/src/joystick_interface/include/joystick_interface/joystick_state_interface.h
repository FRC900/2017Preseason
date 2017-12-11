#pragma once

#include <cassert>
#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>

namespace hardware_interface
{
	// These should mirror the modes listed in ControlModes.h
	// Need a separate copy here though, since sim won't be
	// including that header file - sim shouldn't require
	// anything specifically from the actual motor controller
	// hardware

	// Class which contains state information
	// about a given Joystick SRX. This should include
	// data about the mode the Joystick is running in,
	// current config and setpoint as well as data
	// from the attached encoders, limit switches,
	// etc.
	// Each pass through read() in the low-level
	// hardware interface should update the member
	// vars of this class.
	// The controllers can access the member variables
	// as needed to make decisions in their update code
	// The hardware_controller is responsible for keeping
	// a master array of these classes - 1 entry per
	// physical Joystick controller in the robot
	class JoystickHWState
	{
		public:
			struct ButtonState
			{
       				bool button;
  				bool press;
			        bool release;
			};
			 double leftStickX; //consider changing sticks to vectors
			 double leftStickY;
			 double rightStickX;
			 double rightStickY;
			 double leftTrigger;
			 double rightTrigger;
			 ButtonState buttonX;
			 ButtonState buttonY;
			 ButtonState buttonA;
			 ButtonState buttonB;
			 ButtonState bumperLeft;
			 ButtonState bumperRight;
			 ButtonState buttonBack;
			 ButtonState buttonStart;
			 ButtonState stickLeft;
			 ButtonState stickRight;
			 ButtonState dirLeft;
			 ButtonState dirUp;
			 ButtonState dirRight;
			 ButtonState dirDown;

		private:
	};

	// Handle - used by each controller to get, by name of the
	// corresponding joint, an interface with which to get state
	// info about a Joystick
	class JoystickStateHandle
	{
		public:
			JoystickStateHandle(void) :
				state_(0)
			{}

			// Initialize the base JointStateHandle with pointers
			// from the state data object.  Since the standard ROS
			// code uses JointStateHandles in some places to display
			// robot state support that code as much as possible.  We'll
			// have to figure out what effort maps to in the Joystick
			// Anything above and beyond the 3 standard ROS state
			// vars (position, velocity, effort) will require support
			// in the controller as well as the HWState object pointed
			// to by a given handle.
			JoystickStateHandle(const std::string &name, const JoystickHWState *state) :
				name_(name),
				state_(state)
			{
				if (!state)
					throw HardwareInterfaceException("Cannot create Joystick state handle '" + name + "'. state pointer is null.");
			}
			std::string getName(void) const {return name_;}

			// Operator which allows access to methods from
			// the JoystickHWState member var associated with this
			// handle
			// Note that we could create separate methods in
			// the handle class for every method in the HWState
			// class, e.g.
			//     double getFoo(void) const {assert(_state); return state_->getFoo();}
			// but if each of them just pass things unchanged between
			// the calling code and the HWState method there's no
			// harm in making a single method to do so rather than
			// dozens of getFoo() one-line methods
			const JoystickHWState * operator->() const {assert(state_); return state_;}

		private:
			std::string         name_;
			const JoystickHWState *state_; // leave this const since state should never change the Joystick itself
	};

	// Glue code to let this be registered in the list of
	// hardware resources on the robot.  Since state is
	// read-only, allow multiple controllers to register it.
	class JoystickStateInterface : public HardwareResourceManager<JoystickStateHandle> {};
}
