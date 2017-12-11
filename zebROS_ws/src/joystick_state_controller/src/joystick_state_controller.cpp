///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012, hiDOF INC.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of hiDOF Inc nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/*
 * Author: Wim Meeussen
 */

#include <algorithm>
#include <cstddef>

#include "joystick_state_controller/joystick_state_controller.h"

namespace joystick_state_controller
{

  bool JoystickStateController::init(hardware_interface::JoystickStateInterface* hw,
                                  ros::NodeHandle&                         root_nh,
                                  ros::NodeHandle&                         controller_nh)
  {
    // get all joint names from the hardware interface
    const std::vector<std::string>& joint_names = hw->getNames();
    num_hw_joints_ = joint_names.size();
    for (unsigned i=0; i<num_hw_joints_; i++)
      ROS_DEBUG("Got joint %s", joint_names[i].c_str());

    // get publishing period
    if (!controller_nh.getParam("publish_rate", publish_rate_)){
      ROS_ERROR("Parameter 'publish_rate' not set");
      return false;
    }

    // realtime publisher
    realtime_pub_.reset(new
    realtime_tools::RealtimePublisher<joystick_state_controller::JoystickState>(root_nh, "joystick_states",
    4));

    // get joints and allocate message
    for (unsigned i=0; i<num_hw_joints_; i++){
      realtime_pub_->msg_.name.push_back(joint_names[i]);
      
      realtime_pub_->msg_.leftStickX.push_back(0.0);
      realtime_pub_->msg_.leftStickY.push_back(0.0);
      realtime_pub_->msg_.rightStickX.push_back(0.0);
      realtime_pub_->msg_.rightStickY.push_back(0.0);
      realtime_pub_->msg_.leftTrigger.push_back(0.0);
      realtime_pub_->msg_.rightTrigger.push_back(0.0);

      realtime_pub_->msg_.buttonX.push_back(false);
      realtime_pub_->msg_.buttonXPress.push_back(false);
      realtime_pub_->msg_.buttonXRelease.push_back(false);
      realtime_pub_->msg_.buttonY.push_back(false);
      realtime_pub_->msg_.buttonYPress.push_back(false);
      realtime_pub_->msg_.buttonYRelease.push_back(false);
      realtime_pub_->msg_.buttonA.push_back(false);
      realtime_pub_->msg_.buttonAPress.push_back(false);
      realtime_pub_->msg_.buttonARelease.push_back(false);
      realtime_pub_->msg_.buttonB.push_back(false);
      realtime_pub_->msg_.buttonBPress.push_back(false);
      realtime_pub_->msg_.buttonBRelease.push_back(false);
      realtime_pub_->msg_.buttonStart.push_back(false);
      realtime_pub_->msg_.buttonStartPress.push_back(false);
      realtime_pub_->msg_.buttonStartRelease.push_back(false);
      realtime_pub_->msg_.buttonBack.push_back(false);
      realtime_pub_->msg_.buttonBackPress.push_back(false);
      realtime_pub_->msg_.buttonBackRelease.push_back(false);
      realtime_pub_->msg_.bumperLeft.push_back(false);
      realtime_pub_->msg_.bumperLeftPress.push_back(false);
      realtime_pub_->msg_.bumperLeftRelease.push_back(false);
      realtime_pub_->msg_.bumperRight.push_back(false);
      realtime_pub_->msg_.bumperRightPress.push_back(false);
      realtime_pub_->msg_.bumperRightRelease.push_back(false);
      realtime_pub_->msg_.stickLeft.push_back(false);
      realtime_pub_->msg_.stickLeftPress.push_back(false);
      realtime_pub_->msg_.stickLeftRelease.push_back(false);
      realtime_pub_->msg_.stickRight.push_back(false);
      realtime_pub_->msg_.stickRightPress.push_back(false);
      realtime_pub_->msg_.stickRightRelease.push_back(false);
      realtime_pub_->msg_.dirLeft.push_back(false);
      realtime_pub_->msg_.dirLeftPress.push_back(false);
      realtime_pub_->msg_.dirLeftRelease.push_back(false);
      realtime_pub_->msg_.dirDown.push_back(false);
      realtime_pub_->msg_.dirDownPress.push_back(false);
      realtime_pub_->msg_.dirDownRelease.push_back(false);
      realtime_pub_->msg_.dirUp.push_back(false);
      realtime_pub_->msg_.dirUpPress.push_back(false);
      realtime_pub_->msg_.dirUpRelease.push_back(false);
      realtime_pub_->msg_.dirRight.push_back(false);
      realtime_pub_->msg_.dirRightPress.push_back(false);
      realtime_pub_->msg_.dirRightRelease.push_back(false);
            

      joystick_state_.push_back(hw->getHandle(joint_names[i]));

    }
    addExtraJoints(controller_nh, realtime_pub_->msg_);

    return true;
  }

  void JoystickStateController::starting(const ros::Time& time)
  {
    // initialize time
    last_publish_time_ = time;
  }

  void JoystickStateController::update(const ros::Time& time, const ros::Duration& /*period*/)
  {
    // limit rate of publishing
    if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0/publish_rate_) < time){

      // try to publish
      if (realtime_pub_->trylock()){
        // we're actually publishing, so increment time
        last_publish_time_ = last_publish_time_ + ros::Duration(1.0/publish_rate_);

        // populate joint state message:
        // - fill only joints that are present in the JointStateInterface, i.e. indices [0, num_hw_joints_)
        // - leave unchanged extra joints, which have static values, i.e. indices from num_hw_joints_ onwards
        /*
			double getPosition(void)      const {return position_;}
			double getSpeed(void)         const {return speed_;}
			double getOutputVoltage(void) const {return output_voltage_;}
			int    getCANID(void)         const {return can_id_;}
			double getOutputCurrent(void) const {return output_current_;}
			double getBusVoltage(void)    const {return bus_voltage_;}
			double getPidfP(void)	      const {return pidf_p_;}
			double getPidfI(void)	      const {return pidf_i_;}
			double getPidfD(void)	      const {return pidf_d_;}
			double getPidfF(void)	      const {return pidf_f_;}
			int getClosedLoopError(void)  const {return closed_loop_error_;}
			int getFwdLimitSwitch(void)   const {return fwd_limit_switch_closed_;}
			int getRevLimitSwitch(void)   const {return rev_limit_switch_closed_;}
			JoystickMode getJoystickMode(void)  const {return joystick_mode_;}
            */
        realtime_pub_->msg_.header.stamp = time;
        for (unsigned i=0; i<num_hw_joints_; i++){
	      realtime_pub_->msg_.leftStickX[i] = joystick_state_[i].;
	      realtime_pub_->msg_.leftStickY[i] = joystick_state_[i].;
	      realtime_pub_->msg_.rightStickX[i] = joystick_state_[i].;
	      realtime_pub_->msg_.rightStickY[i] = joystick_state_[i].;
	      realtime_pub_->msg_.leftTrigger[i] = joystick_state_[i].;
	      realtime_pub_->msg_.rightTrigger[i] = joystick_state_[i].;

	      realtime_pub_->msg_.buttonX[i] = joystick_state_[i].;
	      realtime_pub_->msg_.buttonXPress[i] = joystick_state_[i].;
	      realtime_pub_->msg_.buttonXRelease[i] = joystick_state_[i].;
	      realtime_pub_->msg_.buttonY[i] = joystick_state_[i].;
	      realtime_pub_->msg_.buttonYPress[i] = joystick_state_[i].;
	      realtime_pub_->msg_.buttonYRelease[i] = joystick_state_[i].;
	      realtime_pub_->msg_.buttonA[i] = joystick_state_[i].;
	      realtime_pub_->msg_.buttonAPress[i] = joystick_state_[i].;
	      realtime_pub_->msg_.buttonARelease[i] = joystick_state_[i].;
	      realtime_pub_->msg_.buttonB[i] = joystick_state_[i].;
	      realtime_pub_->msg_.buttonBPress[i] = joystick_state_[i].;
	      realtime_pub_->msg_.buttonBRelease[i] = joystick_state_[i].;
	      realtime_pub_->msg_.buttonStart[i] = joystick_state_[i].;
	      realtime_pub_->msg_.buttonStartPress[i] = joystick_state_[i].;
	      realtime_pub_->msg_.buttonStartRelease[i] = joystick_state_[i].;
	      realtime_pub_->msg_.buttonBack[i] = joystick_state_[i].;
	      realtime_pub_->msg_.buttonBackPress[i] = joystick_state_[i].;
	      realtime_pub_->msg_.buttonBackRelease[i] = joystick_state_[i].;
	      realtime_pub_->msg_.bumperLeft[i] = joystick_state_[i].;
	      realtime_pub_->msg_.bumperLeftPress[i] = joystick_state_[i].;
	      realtime_pub_->msg_.bumperLeftRelease[i] = joystick_state_[i].;
	      realtime_pub_->msg_.bumperRight[i] = joystick_state_[i].;
	      realtime_pub_->msg_.bumperRightPress[i] = joystick_state_[i].;
	      realtime_pub_->msg_.bumperRightRelease[i] = joystick_state_[i].;
	      realtime_pub_->msg_.stickLeft[i] = joystick_state_[i].;
	      realtime_pub_->msg_.stickLeftPress[i] = joystick_state_[i].;
	      realtime_pub_->msg_.stickLeftRelease[i] = joystick_state_[i].;
	      realtime_pub_->msg_.stickRight[i] = joystick_state_[i].;
	      realtime_pub_->msg_.stickRightPress[i] = joystick_state_[i].;
	      realtime_pub_->msg_.stickRightRelease[i] = joystick_state_[i].;
	      realtime_pub_->msg_.dirLeft[i] = joystick_state_[i].;
	      realtime_pub_->msg_.dirLeftPress[i] = joystick_state_[i].;
	      realtime_pub_->msg_.dirLeftRelease[i] = joystick_state_[i].;
	      realtime_pub_->msg_.dirDown[i] = joystick_state_[i].;
	      realtime_pub_->msg_.dirDownPress[i] = joystick_state_[i].;
	      realtime_pub_->msg_.dirDownRelease[i] = joystick_state_[i].;
	      realtime_pub_->msg_.dirUp[i] = joystick_state_[i].;
	      realtime_pub_->msg_.dirUpPress[i] = joystick_state_[i].;
	      realtime_pub_->msg_.dirUpRelease[i] = joystick_state_[i].;
	      realtime_pub_->msg_.dirRight[i] = joystick_state_[i].;
	      realtime_pub_->msg_.dirRightPress[i] = joystick_state_[i].;
	      realtime_pub_->msg_.dirRightRelease[i] = joystick_state_[i].;
          }
        }
        realtime_pub_->unlockAndPublish();
      }
    }
  }

  void JoystickStateController::stopping(const ros::Time& /*time*/)
  {}

  void JoystickStateController::addExtraJoints(const ros::NodeHandle& nh,
  joystick_state_controller::JoystickState& msg)
  {

    // Preconditions
    XmlRpc::XmlRpcValue list;
    if (!nh.getParam("extra_joints", list))
    {
      ROS_DEBUG("No extra joints specification found.");
      return;
    }

    if (list.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("Extra joints specification is not an array. Ignoring.");
      return;
    }

    for(int i = 0; i < list.size(); ++i)
    {
      if (list[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
      {
        ROS_ERROR_STREAM("Extra joint specification is not a struct, but rather '" << list[i].getType() <<
                         "'. Ignoring.");
        continue;
      }

      if (!list[i].hasMember("name"))
      {
        ROS_ERROR_STREAM("Extra joint does not specify name. Ignoring.");
        continue;
      }

      const std::string name = list[i]["name"];
      if (std::find(msg.name.begin(), msg.name.end(), name) != msg.name.end())
      {
        ROS_WARN_STREAM("Joint state interface already contains specified extra joint '" << name << "'.");
        continue;
      }

    }
  }

}

PLUGINLIB_EXPORT_CLASS( joystick_state_controller::JoystickStateController, controller_interface::ControllerBase)
