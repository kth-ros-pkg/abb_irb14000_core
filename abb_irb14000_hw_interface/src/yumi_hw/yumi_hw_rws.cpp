#include <yumi_hw/yumi_hw_rws.h>


bool YumiJointStateHandler::init(industrial::smpl_msg_connection::SmplMsgConnection* connection)
{
  first_iteration_ = true;
  joint_state_received_ = false;
  joint_commands_set_ = false;

  return init((int)industrial::simple_message::StandardMsgTypes::JOINT, connection);
}

bool YumiJointStateHandler::getJointStates(float (&jnts)[N_YUMI_JOINTS])
{
  boost::mutex::scoped_lock lock(data_buffer_mutex_);
  while(!joint_state_received_) 
  {
      c_joint_state_received_.wait(lock);
  }
  joint_state_received_ = false;
  memcpy(&jnts, &joint_positions_, sizeof(jnts));
}

bool YumiJointStateHandler::setJointCommands(float (&jnts)[N_YUMI_JOINTS], int mode)
{
  boost::mutex::scoped_lock lock(data_buffer_mutex_);
  memcpy(&joint_command_, &jnts, sizeof(jnts));
  joint_commands_set_ = true;
  mode_ = mode;
  c_joint_commands_set_.notify_all();
}

bool YumiJointStateHandler::internalCB(industrial::simple_message::SimpleMessage& in)
{
  boost::mutex::scoped_lock lock(data_buffer_mutex_);
  
  //ROS_INFO("Received a message");
  industrial::joint_message::JointMessage joint_msg;
  bool rtn = true;

  if (!joint_msg.init(in))
  {
    ROS_ERROR("Failed to initialize joint message");
    return false;
  }

  //ROS_INFO("Message parsed");
  industrial::shared_types::shared_real joint_value_from_msg;

  for(int i=0; i<N_YUMI_JOINTS; i++) 
  {
    //std::cerr<<i<<":";
    if (joint_msg.getJoints().getJoint(i, joint_value_from_msg))
    {
        joint_positions_[i] = joint_value_from_msg;
        //std::cerr<<joint_positions_[i]<<" ";
    }
    else
    {
        //std::cerr<<"X";
        rtn = false;
    }
  }
  //std::cerr<<"\n";

  // Reply back to the controller if the sender requested it.
  if (industrial::simple_message::CommTypes::SERVICE_REQUEST == joint_msg.getMessageType())
  {
    //ROS_INFO("Reply requested, sending");
    industrial::simple_message::SimpleMessage reply;
    joint_msg.toReply(reply, rtn ? industrial::simple_message::ReplyTypes::SUCCESS 
                                 : industrial::simple_message::ReplyTypes::FAILURE);
    this->getConnection()->sendMsg(reply);
  }

  joint_state_received_ = true;
  c_joint_state_received_.notify_all();

  while(!joint_commands_set_)
  {
    c_joint_commands_set_.wait(lock);
  }
  joint_commands_set_ = false;

  //if first call back, then mirror state to command
  if(first_iteration_) 
  {
    ROS_INFO("Mirroring to command");
    memcpy(&joint_command_, &joint_positions_, sizeof(joint_command_));
    first_iteration_ = false;	
  }
  
  //TODO: format trajectory request message
  industrial::shared_types::shared_real joint_value_to_msg;

  for(int i=0; i<N_YUMI_JOINTS; i++) 
  {
    joint_value_to_msg = joint_command_[i];
//		std::cerr<<joint_command_[i]<<" ";
    if (!joint_msg.getJoints().setJoint(i, joint_value_to_msg))
    {
        rtn = false;
    }
  }
  
  if (!joint_msg.getJoints().setJoint(N_YUMI_JOINTS, mode_))
  {
    rtn = false;
  }
//	    std::cerr<<"\n";

  //TODO: send back on conncetion 
  industrial::simple_message::SimpleMessage next_point;
  joint_msg.toRequest(next_point);
  this->getConnection()->sendMsg(next_point);

  //ROS_INFO("Done processing");
  
  return rtn;
}




YumiRwsInterface::YumiRwsInterface() 
{
  this->connection_ = NULL;
  stop_comm_ = true;
}

YumiRwsInterface::~YumiRwsInterface() 
{ 
  stopThreads();
}

bool YumiRwsInterface::init(std::string ip, int port) 
{
  // initialize connection 
  char* ip_addr = strdup(ip.c_str());  // connection.init() requires "char*", not "const char*"
  ROS_INFO("Robot state connecting to IP address: '%s:%d'", ip_addr, port);
  default_tcp_connection_.init(ip_addr, port);
  free(ip_addr);

  connection_ = &default_tcp_connection_;
  connection_->makeConnect();
  ROS_INFO("Connection established");

  // initialize message manager
  manager_.init(connection_);

  // initialize message handler
  js_handler_.init(connection_);

  // register handler to manager
  manager_.add(&js_handler_, false);

  ROS_INFO("Callbacks and handlers set up");
  stop_comm_ = false;
}

void YumiRwsInterface::stopThreads() 
{
  stop_comm_ = true;
  rws_comm_thread_.join();
}

void YumiRwsInterface::startThreads() 
{
  if(!stop_comm_) 
  {
      boost::thread(boost::bind(&YumiRwsInterface::rwsCommThreadCallback,this ));
  }
}

void YumiRwsInterface::getCurrentJointStates(float (&joints)[N_YUMI_JOINTS]) 
{
  js_handler_.getJointStates(joints);	    
}

void YumiRwsInterface::setJointTargets(float (&joints)[N_YUMI_JOINTS], int mode) 
{
  js_handler_.setJointCommands(joints, mode);
}

void YumiRwsInterface::rwsCommThreadCallback()
{
  while(!stop_comm_) 
  {
      manager_.spinOnce();
  }
  return;
}




YumiHwRws::YumiHwRws(const double& exponential_smoothing_alpha)
  :
    YumiHW(exponential_smoothing_alpha),
    is_initialized_(false),
    is_setup_(false),
    first_run_in_position_mode_(true)
{ 
  
}

YumiHwRws::~YumiHwRws() { 
    rws_interface_.stopThreads();
}

void YumiHwRws::setup(std::string ip, int port) 
{
  ip_ = ip;
  port_ = port;
  is_setup_ = true;
}

bool YumiHwRws::init()
{
  if (is_initialized_)
  {
    return false;
  }

  if(!is_setup_) 
  {
    ROS_ERROR("IP and port of controller are not set up!");
    return false;
  }

  rws_interface_.init(ip_, port_);
  rws_interface_.startThreads();
  is_initialized_ = true;

  return true;
}	

void YumiHwRws::read(ros::Time time, ros::Duration period)
{
  if(!is_initialized_) 
  {
    return;
  }

  //ROS_INFO("reading joints");
  data_buffer_mutex_.lock();
  rws_interface_.getCurrentJointStates(read_joint_position_);

  for (int j = 0; j < n_joints_; j++)
  {
    joint_position_prev_[j] = joint_position_[j];
    joint_position_[j] = read_joint_position_[j];
    //joint_effort_[j] = readJntEffort[j]; //TODO: read effort 
    joint_velocity_[j] = filters::exponentialSmoothing((joint_position_[j] - joint_position_prev_[j]) / period.toSec(),
                                                        joint_velocity_[j], exponential_smoothing_alpha_);
    if(first_run_in_position_mode_)
    {
      joint_position_command_[j] = read_joint_position_[j];
    }
  }
  first_run_in_position_mode_ = false;
  data_buffer_mutex_.unlock();
  //ROS_INFO("read joints");

  return;
}

void YumiHwRws::write(ros::Time time, ros::Duration period)
{
  if(!is_initialized_) return;  
  
  enforceLimits(period);

  //ROS_INFO("writing joints");
  data_buffer_mutex_.lock();
  switch (getControlStrategy())
  {
    case JOINT_POSITION:
      for (int j = 0; j < n_joints_; j++)
      {
        new_joint_position_[j] = joint_position_command_[j];
      }	
      break;

    case JOINT_VELOCITY:
      //std::cerr<<"ASS = ";
      for (int j = 0; j < n_joints_; j++)
      {
        //std::cerr<<joint_velocity_command_[j]<<"*"<<period.toSec()<<" + "<<joint_position_[j]<<" ::: ";  
        new_joint_position_[j] = joint_velocity_command_[j]; //*period.toSec() + joint_position_[j]; 
      }
      //std::cerr<<std::endl;
      first_run_in_position_mode_ = true;
      break;
      //case JOINT_EFFORT:
      //break;
    default: 
      break;
  }

  rws_interface_.setJointTargets(new_joint_position_, getControlStrategy());
  data_buffer_mutex_.unlock();
  //ROS_INFO("wrote joints");

  return;
}

