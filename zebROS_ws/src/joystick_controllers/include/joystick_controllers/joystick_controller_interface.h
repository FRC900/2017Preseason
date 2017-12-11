
#pragma once
#include <dynamic_reconfigure/server.h>
#include <joystick_interface/joystick_command_interface.h>

namespace joystick_controllers
{
// Create a wrapper class for each Joystick mode.  For the basic controllers
// this isn't really helpful since the code could just be included in the
// controller itself.  But consider a more complex controller, for example
// swerve. The swerve controller runs a number of wheels, and each wheel
// has both position and velocity.  The wheel class might create a
// JoystickPosisionPIDControllerInterface member var for the position moter and
// also a JoystickVelocityPIDControllerInterface member var for the velocity.
// And since it will be creating one per wheel, it makes sense to wrap the common
// init code into a class rather than duplicate it for each wheel. Another
// controller - say a shooter wheel - could also use this same code to
// create a joystick handle to access that motor
//

// Class which provides a common set of code for reading
// parameters for motor controllers from yaml / command line
// ROS params.  Not all of these values will be needed for all
// modes - specific controller interfaces will use what
// they do need and ignore the rest.
// The idea here is that code using a particular CI would
// call readParams(), modify any parameters which are specific
// to the controller, and then call init using the specificed
// parameters. This will handle the common case where most
// code using the CI will want to use the default names of settings
// but also allow customization
class JoystickCIParams
{
	public:
		// Initialize with relatively sane defaults
		// for all parameters

		// Read a joint name from the given nodehandle's params
		bool readJointName(ros::NodeHandle &n, const std::string &param_name)
		{
			if (!n.getParam(param_name, joint_name_))
			{
				ROS_ERROR("No joint given (namespace: %s, param name: %s)", 
						  n.getNamespace().c_str(), param_name.c_str());
				return false;
			}
			return true;
		}

	private:
		// Read a double named <param_type> from the array/map
		// in params
		double findDoubleParam(std::string param_type, XmlRpc::XmlRpcValue &params) const
		{
			if (!params.hasMember(param_type))
				return 0;
			XmlRpc::XmlRpcValue& param = params[param_type];
			if (!param.valid())
				throw std::runtime_error(param_type + " was not a double valid type");
			if (param.getType() == XmlRpc::XmlRpcValue::TypeDouble)
				return (double)param;
			else if (param.getType() == XmlRpc::XmlRpcValue::TypeInt)
				return (int)param;
			else
				throw std::runtime_error("A non-double value was passed for" + param_type);
			return 0;
		}

		// Read a bool named <param_type> from the array/map
		// in params
		bool findBoolParam(std::string param_type, XmlRpc::XmlRpcValue &params) const
		{
			if (!params.hasMember(param_type))
				return false;
			XmlRpc::XmlRpcValue& param = params[param_type];
			if (!param.valid())
				throw std::runtime_error(param_type + " was not a bool valid type");
			if (param.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
				return (bool)param;
			else
				throw std::runtime_error("A non-bool value was passed for" + param_type);
			return false;
		}
};

// Base class for controller interface types. This class
// will be the least restrictive - allow users to swtich modes,
// reprogram any config values, and so on.
// Derived classes will be more specialized - they'll only allow
// a specific Joystick mode and disable code which doesn't apply
// to that mode
class JoystickControllerInterface
{
	public:
		// Standardize format for reading params for 
		// motor controller
		bool readParams(ros::NodeHandle &n, hardware_interface::JoystickStateInterface *tsi)
		{
			return params_.readJointName(n, "joint") && 
		}


		// Allow users of the ControllerInterface to get 
		// a copy of the parameters currently set for the
		// Joystick.  They can then modify them at will and
		// call initWithParams to reprogram the Joystick.
		// Hopefully this won't be needed all that often ...
		// the goal should be to provide a simpler interface
		// for commonly used operations
		JoystickCIParams getParams(void) const
		{
			return params_;
		}

		// Initialize Joystick hardware with the settings in params
		bool initWithParams(hardware_interface::JoystickCommandInterface *tci, 
									const JoystickCIParams &params)
		{
			joystick_ = tci->getHandle(params.joint_name_);
			return writeParamsToHW(params);
		}

		// Use data in params_ to actually set up Joystick
		// hardware. Make this a separate method outside of
		// init() so that dynamic reconfigure callback can write
		// values using this method at any time


		// Read params from config file and use them to 
		// initialize the Joystick hardware
		// Useful for the hopefully common case where there's 
		// no need to modify the parameters after reading
		// them
		bool initWithNode(hardware_interface::JoystickCommandInterface *tci,
							 	  hardware_interface::JoystickStateInterface *tsi,
								  ros::NodeHandle &n)
		{
			// Read params from startup and intialize
			// Joystick using them
			bool result = readParams(n, tsi) && initWithParams(tci, params_);

			// Create dynamic_reconfigure Server. Pass in n
			// so that all the vars for the class are grouped
			// under the node's name.  Doing so allows multiple
			// copies of the class to be started, each getting
			// their own namespace.
			srv_ = std::make_shared<dynamic_reconfigure::Server<joystick_controllers::JoystickConfigConfig>>(n);

			// Register a callback function which is run each
			// time parameters are changed using 
			// rqt_reconfigure or the like
			srv_->setCallback(boost::bind(&JoystickControllerInterface::callback, this, _1, _2));	

			return result;
		}

		// Set the setpoint for the motor controller
		void setRumble(double command, int which)
		{
			joystick_->setRumble(which, command);
		}

		
	protected:
		hardware_interface::JoystickCommandHandle joystick_;
		JoystickCIParams                          params_;
		std::shared_ptr<dynamic_reconfigure::Server<joystick_controllers::JoystickConfigConfig>> srv_;

};


} // namespace
