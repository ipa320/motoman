#include "motoman_driver/industrial_robot_client/joint_feedback_ex_relay_handler.h"
#include "simple_message/log_wrapper.h"

using industrial::joint_data::JointData;
using industrial::shared_types::shared_real;
using namespace industrial::simple_message;

namespace industrial_robot_client
{
namespace joint_feedback_ex_relay_handler
{

bool JointFeedbackExRelayHandler::init(SmplMsgConnection* connection,
                                     std::vector<std::string> &joint_names)
{
  bool rtn = JointRelayHandler::init(connection, (int)StandardMsgTypes::JOINT_FEEDBACK_EX, joint_names);

  // try to read robot_id parameter, if none specified
  if ( (robot_id_ < 0) )
    node_.param("robot_id", robot_id_, 0);

  return rtn;
}


bool JointFeedbackExRelayHandler::create_messages(SimpleMessage& msg_in,
                                                control_msgs::FollowJointTrajectoryFeedback* control_state,
                                                sensor_msgs::JointState* sensor_state)
{
  // inspect robot_id field first, to avoid "Failed to Convert" message
  JointFeedbackMessage tmp_msg;
  if (tmp_msg.init(msg_in) && (tmp_msg.getRobotID() != robot_id_))
  {
    LOG_COMM("Ignoring Message: robotID (%d) doesn't match expected (%d)",
             tmp_msg.getRobotID(), robot_id_);
    return false;
  }

  return JointRelayHandler::create_messages(msg_in, control_state, sensor_state);
}

bool JointFeedbackExRelayHandler::convert_message(SimpleMessage& msg_in, JointTrajectoryPoint* joint_state)
{
  JointFeedbackMessage joint_feedback_msg;

  if (!joint_feedback_msg.init(msg_in))
  {
    LOG_ERROR("Failed to initialize joint feedback message");
    return false;
  }

  return convert_message(joint_feedback_msg, joint_state);
}

bool JointFeedbackExRelayHandler::JointDataToVector(const JointData &joints,
                                                  std::vector<double> &vec,
                                                  int len)
{
  if ( (len<0) || (len>joints.getMaxNumJoints()) )
  {
    LOG_ERROR("Failed to copy JointData.  Len (%d) out of range (0 to %d)",
              len, joints.getMaxNumJoints());
    return false;
  }

  vec.resize(len);
  for (int i=0; i<len; ++i)
    vec[i] = joints.getJoint(i);

  return true;
}

bool JointFeedbackExRelayHandler::convert_message(JointFeedbackMessage& msg_in, JointTrajectoryPoint* joint_state)
{
  JointData values;
  int num_jnts = all_joint_names_.size();

  // copy position data
  if (msg_in.getPositions(values))
  {
    if (!JointDataToVector(values, joint_state->positions, num_jnts))
    {
      LOG_ERROR("Failed to parse position data from JointFeedbackMessage");
      return false;
    }
  } else
    joint_state->positions.clear();

  // copy velocity data
  if (msg_in.getVelocities(values))
  {
    if (!JointDataToVector(values, joint_state->velocities, num_jnts))
    {
      LOG_ERROR("Failed to parse velocity data from JointFeedbackMessage");
      return false;
    }
  } else
    joint_state->velocities.clear();

  // copy acceleration data
  if (msg_in.getAccelerations(values))
  {
    if (!JointDataToVector(values, joint_state->accelerations, num_jnts))
    {
      LOG_ERROR("Failed to parse acceleration data from JointFeedbackMessage");
      return false;
    }
  } else
    joint_state->accelerations.clear();

  // copy timestamp data
  shared_real value;
  if (msg_in.getTime(value))
    joint_state->time_from_start = ros::Duration(value);
  else
    joint_state->time_from_start = ros::Duration(0);

  return true;
}

}//namespace joint_feedback_ex_relay_handler
}//namespace industrial_robot_client





