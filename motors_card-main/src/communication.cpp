
#include <micro_ros_platformio.h>

#include "pins.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc_parameter/rclc_parameter.h>
#include <rmw_microros/rmw_microros.h>
#include "hardware/watchdog.h"


#include <std_msgs/msg/int32.h>
#include <std_srvs/srv/set_bool.h>
#include <std_srvs/srv/trigger.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/pose2_d.h>

#include "communication.h"
#include "odometry.h"
#include "regulator.h"
#include "motor.h"
#include "params.h"
#include "clamp_servos.h"

#include <string>

rcl_node_t node;

rcl_allocator_t allocator;

rclc_support_t support;

rclc_parameter_server_t param_server;

const rclc_parameter_options_t options = {
    .notify_changed_over_dds = true,
    .max_params = 6,
    .allow_undeclared_parameters = false,
    .low_mem_mode = false };

rclc_executor_t executor;



rcl_service_t m_resetErrService;
rcl_service_t m_enablePwmService;

rcl_subscription_t m_clampsSubscriber;
rcl_subscription_t m_cmdVelSubscriber;
rcl_subscription_t m_setPoseSubscriber;
rcl_subscription_t m_enablePwmSubscriber;

rcl_publisher_t m_odomPublisher;
rcl_publisher_t m_statusPublisher;

void publish_status(int32_t status){
  std_msgs__msg__Int32 msg;

  msg.data = status;

  rcl_publish(&m_statusPublisher, &msg, NULL);
}

void publish_odom(Odom odom){
  geometry_msgs__msg__Pose2D msg;

  msg.theta = odom.theta;
  msg.x = odom.x;
  msg.y = odom.y;

  rcl_publish(&m_odomPublisher, &msg, NULL);
}


void clampsControlSubscriber_cb(const void * msgin){
  	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;

    if (msg->data == 2){ // CLAMP CLOSE
      open_clamps();
    }else if(msg->data == 1){// CLAMP OPEN
      close_clamps();
    }else if(msg->data == 3){// SERVOS OFF
      activate_dance_mode();
    }

}


void enablePwmSubscriber_cb(const void * msgin){
  	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;

    if (msg->data == 1){
      enable_pwm_motor_right(true);
      enable_pwm_motor_left(true);

    }else{
      enable_pwm_motor_right(false);
      enable_pwm_motor_left(false);
    }

}


void cmdVelSubscriber_cb(const void * msgin)
{
	  geometry_msgs__msg__Twist * msg = (geometry_msgs__msg__Twist *)msgin;
	 
    double lin_vel = msg->linear.x;
    double ang_vel = msg->angular.z;

    set_speed_target(lin_vel, ang_vel);

}

void setPoseSubscriber_cb(const void * msgin)
{
	const geometry_msgs__msg__Pose2D * msg = (const geometry_msgs__msg__Pose2D *)msgin;
	
    Odom new_pos;
    new_pos.x = msg->x;
    new_pos.y = msg->y;
    new_pos.theta = msg->theta;
    set_position(new_pos);

}

bool on_parameter_changed(const Parameter * old_param, const Parameter * new_param, void * context)
{
  (void) context;

  if (old_param == NULL && new_param == NULL) {
    return false;
  }

  if (new_param == NULL) {
    return false;
  } else {
    std::string s = new_param->name.data;
    if(s.compare("mccd.kp.left") == 0){
      PIDparams params = get_left_motor_pid_params();
      params.Kp = new_param->value.double_value;
      set_left_motor_pid_params(params);
    }
    else if(s.compare("mccd.ki.left") == 0)
    {
      PIDparams params = get_left_motor_pid_params();
      params.Ki = new_param->value.double_value;
      set_left_motor_pid_params(params);
    }
    else if(s.compare("mccd.kp.right") == 0)
    {
      PIDparams params = get_right_motor_pid_params();
      params.Kp = new_param->value.double_value;
      set_right_motor_pid_params(params);
    }
    else if(s.compare("mccd.ki.right") == 0)
    {
      PIDparams params = get_right_motor_pid_params();
      params.Ki = new_param->value.double_value;
      set_right_motor_pid_params(params);
    }
    else if(s.compare("mccd.wheel_separation") == 0)
    {
      Geometry geometry = get_geometry();
      geometry.distance_between_encoders = new_param->value.double_value;
      set_geometry(geometry);
    }
    else if(s.compare("mccd.wheel_diameter") == 0)
    {
      Geometry geometry = get_geometry();
      geometry.left_encoder_conversion_factor = TWO_PI*new_param->value.double_value/(1000*4);
      geometry.right_encoder_conversion_factor = TWO_PI*new_param->value.double_value/(1000*4);
      set_geometry(geometry);
    }
    else
    {
      return false;
    }
    printf("\n");
  }

  return true;
}

void init_parameters(){

    rclc_parameter_server_init_with_option(&param_server, &node, &options);
    rclc_executor_add_parameter_server(&executor, &param_server, on_parameter_changed);


    rclc_add_parameter(&param_server, "mccd.kp.left", RCLC_PARAMETER_DOUBLE);
    rclc_add_parameter(&param_server, "mccd.ki.left", RCLC_PARAMETER_DOUBLE);
    rclc_add_parameter(&param_server, "mccd.kp.right", RCLC_PARAMETER_DOUBLE);
    rclc_add_parameter(&param_server, "mccd.ki.right", RCLC_PARAMETER_DOUBLE);
    rclc_add_parameter(&param_server, "mccd.wheel_separation", RCLC_PARAMETER_DOUBLE);
    rclc_add_parameter(&param_server, "mccd.wheel_diameter", RCLC_PARAMETER_DOUBLE);

    rclc_parameter_set_double(&param_server, "mccd.kp.left", 300.0);
    rclc_parameter_set_double(&param_server, "mccd.ki.left", 8.0);
    rclc_parameter_set_double(&param_server, "mccd.kp.right", 200.0);
    rclc_parameter_set_double(&param_server, "mccd.ki.right", 8.0);
    rclc_parameter_set_double(&param_server, "mccd.wheel_separation", 0.35);
    rclc_parameter_set_double(&param_server, "mccd.wheel_diameter", 0.031);
}


geometry_msgs__msg__Twist msg_cmdVel;

void init_communication(){

    allocator = rcl_get_default_allocator();

    set_microros_serial_transports(Serial);

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "mccd", "", &support);

    executor = rclc_executor_get_zero_initialized_executor();

    rclc_executor_init(&executor, &support.context, RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES, &allocator);

    init_parameters();

    rclc_subscription_init_default(
		&m_enablePwmSubscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"/mccd/enablePwm");
    std_msgs__msg__Int32 msg_bool;
    rclc_executor_add_subscription(&executor, &m_enablePwmSubscriber, &msg_bool, &enablePwmSubscriber_cb, ON_NEW_DATA);
    
    rclc_subscription_init_default(
		&m_clampsSubscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"/mccd/clampsControl");
    std_msgs__msg__Int32 msg_clampCtrl;
    rclc_executor_add_subscription(&executor, &m_clampsSubscriber, &msg_clampCtrl, &clampsControlSubscriber_cb, ON_NEW_DATA);

    rclc_subscription_init_best_effort(
		&m_cmdVelSubscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
		"/mccd/cmdVel");
    
    rclc_executor_add_subscription(&executor, &m_cmdVelSubscriber, &msg_cmdVel, &cmdVelSubscriber_cb, ON_NEW_DATA);

    rclc_subscription_init_default(
		&m_setPoseSubscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose2D),
		"/mccd/setPose");
    geometry_msgs__msg__Pose2D msg_setPose;
    rclc_executor_add_subscription(&executor, &m_setPoseSubscriber, &msg_setPose, &setPoseSubscriber_cb, ON_NEW_DATA);



	  rclc_publisher_init_default(
		&m_odomPublisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose2D),
		"/mccd/odom");

    rclc_publisher_init_default(
		&m_statusPublisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"/mccd/status");
    

    ros_spin_once_communication();

    //rmw_uros_ping_agent();
}

void check_and_reset_for_ros_agent_connected(){
  rcl_ret_t ret = rmw_uros_ping_agent(500, 2);
    if (ret != RCL_RET_OK)
    {
        watchdog_enable(1, 1);
    }  
}

void ros_spin_once_communication(){
  rclc_executor_spin_some(&executor, 100000);
}

void deinit_communication(){
    
    rcl_service_fini(&m_enablePwmService, &node);
    rcl_service_fini(&m_resetErrService, &node);

    rcl_subscription_fini(&m_cmdVelSubscriber, &node);
    rcl_subscription_fini(&m_setPoseSubscriber, &node);
    rcl_subscription_fini(&m_clampsSubscriber, &node);

    rcl_publisher_fini(&m_odomPublisher, &node);
    rcl_publisher_fini(&m_statusPublisher, &node);

    rcl_node_fini(&node);
};
