#ifndef JOINT_FEEDBACK_EX_RELAY_HANDLER_H
#define JOINT_FEEDBACK_EX_RELAY_HANDLER_H


#include "motoman_driver/industrial_robot_client/joint_relay_handler.h"
#include "simple_message/messages/joint_feedback_message.h"

namespace industrial_robot_client
{
namespace joint_feedback_ex_relay_handler
{

using industrial::joint_feedback_message::JointFeedbackMessage;
using industrial::simple_message::SimpleMessage;
using industrial::smpl_msg_connection::SmplMsgConnection;
using industrial_robot_client::joint_relay_handler::JointRelayHandler;
using trajectory_msgs::JointTrajectoryPoint;

/**
 * \brief Message handler that relays joint positions (converts simple message
 * types to ROS message types and publishes them)
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class JointFeedbackExRelayHandler : public industrial_robot_client::joint_relay_handler::JointRelayHandler
{

public:

  /**
* \brief Constructor
*/
  JointFeedbackExRelayHandler(int robot_id=-1) : robot_id_(robot_id) {};


 /**
  * \brief Class initializer
  *
  * \param connection simple message connection that will be used to send replies.
  * \param joint_names list of joint-names for msg-publishing.
  *   - Count and order should match data from robot connection.
  *   - Use blank-name to exclude a joint from publishing.
  *
  * \return true on success, false otherwise (an invalid message type)
  */
 virtual bool init(SmplMsgConnection* connection,
                    std::vector<std::string> &joint_names);

protected:
 int robot_id_;

  /**
   * \brief Convert joint message into intermediate message-type
   *
   * \param[in] msg_in Message from robot connection
   * \param[out] joint_state JointTrajectoryPt message for intermediate processing
   */
  virtual bool convert_message(SimpleMessage& msg_in, JointTrajectoryPoint* joint_state);

  // override JointRelayHandler::create_messages, to check robot_id w/o error msg
  bool create_messages(SimpleMessage& msg_in,
                       control_msgs::FollowJointTrajectoryFeedback* control_state,
                       sensor_msgs::JointState* sensor_state);
private:

  static bool JointDataToVector(const industrial::joint_data::JointData &joints,
                                std::vector<double> &vec, int len);

  /**
   * \brief Convert joint feedback message into intermediate message-type
   *
   * \param[in] msg_in JointFeedbackMessage from robot connection
   * \param[out] joint_state JointTrajectoryPt message for intermediate processing
   */
  bool convert_message(JointFeedbackMessage& msg_in, JointTrajectoryPoint* joint_state);

};//class JointFeedbackExRelayHandler

}//joint_feedback_ex_relay_handler
}//industrial_robot_cliet


#endif // JOINT_FEEDBACK_EX_RELAY_HANDLER_H
