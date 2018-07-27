#ifndef YUMI_HW_RWS_H
#define YUMI_HW_RWS_H

#include "yumi_hw/yumi_hw.h"

#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include "simple_message/message_handler.h"
#include "simple_message/message_manager.h"
#include "simple_message/messages/joint_message.h"
#include "simple_message/smpl_msg_connection.h"
#include "simple_message/socket/tcp_socket.h"
#include "simple_message/socket/tcp_client.h"



/**
  * Overrides message handler: keeps joint states thread-safe.
  */
class YumiJointStateHandler : public industrial::message_handler::MessageHandler
{
public:
  using industrial::message_handler::MessageHandler::init;

  bool init(industrial::smpl_msg_connection::SmplMsgConnection* connection);
  
  bool getJointStates(float (&jnts)[N_YUMI_JOINTS]);

  bool setJointCommands(float (&jnts)[N_YUMI_JOINTS], int mode);

protected:
  bool internalCB(industrial::simple_message::SimpleMessage& in);

private:
  int mode_;

  bool first_iteration_;

  float joint_positions_[N_YUMI_JOINTS];
  float joint_command_[N_YUMI_JOINTS];

  bool joint_state_received_;
  bool joint_commands_set_;

  boost::mutex data_buffer_mutex_;
  boost::condition_variable c_joint_state_received_;
  boost::condition_variable c_joint_commands_set_;
};



/**
  * Keep a connection to the robot and send and receive joint states
  */
class YumiRwsInterface {
public:
  YumiRwsInterface();

  ~YumiRwsInterface();
  
  bool init(std::string ip = "", 
            int port = industrial::simple_socket::StandardSocketPorts::STATE);

  void stopThreads();

  void startThreads();

  void getCurrentJointStates(float (&joints)[N_YUMI_JOINTS]);

  void setJointTargets(float (&joints)[N_YUMI_JOINTS], int mode);

private:

  virtual void rwsCommThreadCallback();

  YumiJointStateHandler js_handler_;

  bool stop_comm_;
  boost::thread rws_comm_thread_;
  
  // Connection over ros industrial sockets
  industrial::tcp_client::TcpClient default_tcp_connection_; //?
  //industrial::tcp_client::RobotStatusRelayHandler default_robot_status_handler_; //?
  industrial::smpl_msg_connection::SmplMsgConnection* connection_;
  industrial::message_manager::MessageManager manager_;
};



/**
  * RobotHW interface class that connects to YuMi over RWS
  */
class YumiHwRws : public YumiHW
{
public:
  YumiHwRws(const double& exponential_smoothing_alpha = 0.04);

  ~YumiHwRws();
  
  void setup(std::string ip = "", 
             int port = industrial::simple_socket::StandardSocketPorts::STATE);

  // Init, read, and write, with FRI hooks
  bool init();

  /// Copies the last received joint state out to the controller manager
  void read(ros::Time time, ros::Duration period);

  /// Caches the most recent joint commands into the robot interface  
  void write(ros::Time time, ros::Duration period);

private:

  ///
  YumiRwsInterface rws_interface_;

  std::string ip_;
  int port_;

  bool is_initialized_;
  bool is_setup_;
  bool first_run_in_position_mode_;
  
  boost::mutex data_buffer_mutex_;
  
  ///command buffers
  float new_joint_position_[N_YUMI_JOINTS];
  ///data buffers
  float read_joint_position_[N_YUMI_JOINTS];
  float last_comm_[N_YUMI_JOINTS];


};

#endif // YUMI_HW_RWS_H
