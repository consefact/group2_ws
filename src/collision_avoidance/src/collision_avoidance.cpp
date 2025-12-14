#include <collision_avoidance.h>

// 全局变量定义
int mission_num = 0;
float if_debug = 0;
float err_max = 0.2;
const float HOVER_DURATION = 10.0f; // 降落悬停时间（秒）
void print_param()
{
  std::cout << "=== 控制参数 ===" << std::endl;
  std::cout << "err_max: " << err_max << std::endl;
  std::cout << "ALTITUDE: " << ALTITUDE << std::endl;
  std::cout << "if_debug: " << if_debug << std::endl;
  if(if_debug == 1) cout << "自动offboard" << std::endl;
  else cout << "遥控器offboard" << std::endl;
}


int main(int argc, char **argv)
{
  // 防止中文输出乱码
  setlocale(LC_ALL, "");

  // 初始化ROS节点
  ros::init(argc, argv, "collision_avoidance");
  ros::NodeHandle nh;

  // 订阅mavros相关话题
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);

  // 发布无人机多维控制话题
  ros::Publisher mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);
  ros::Subscriber livox_sub = nh.subscribe<livox_ros_driver::CustomMsg>("/livox/lidar", 10, livox_custom_cb);

  // 创建服务客户端
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  ros::ServiceClient ctrl_pwm_client = nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");

  // 设置话题发布频率，需要大于2Hz，飞控连接有500ms的心跳包
  ros::Rate rate(20);

  // 参数读取

  nh.param<float>("err_max", err_max, 0);
  nh.param<float>("if_debug", if_debug, 0);
  nh.param<double>("zero_plane_height", zero_plane_height, 0);
  nh.param<double>("height_threshold", height_threshold, 0.05);
  nh.param<double>("min_range", min_range, 0.1);
  nh.param<double>("max_range", max_range, 30.0);
  nh.param<int>("num_bins", num_bins, 360);

  nh.param<float>("R_outside", R_outside, 0.0);
  nh.param<float>("R_inside", R_inside, 0.0);
  nh.param<float>("p_R", p_R, 0.0);
  nh.param<float>("p_r", p_r, 0.0);
  nh.param<float>("p_xy", p_xy, 0.0);
  nh.param<float>("vel_collision_max", vel_collision_max, 0.0);
  nh.param<float>("vel_track_max", vel_track_max, 0.0);
  nh.param<float>("vel_sp_max", vel_sp_max, 0.0);
  nh.param<float>("target_x", target.x, 0.0);
  nh.param<float>("target_y", target.y, 0.0);
  nh.param<float>("mission_cruise_timeout", mission_cruise_timeout, mission_cruise_timeout);       // 读取普通巡航，第七次
  nh.param<float>("collision_cruise_timeout", collision_cruise_timeout, collision_cruise_timeout); // 读取避障巡航，第七次
  nh.param<float>("final_r", final_r,final_r); // 读取终点限制圆半径
  ros::Timer timer1 = nh.createTimer(ros::Duration(2.0), time_c_b_pos);
  ros::Timer timer2 = nh.createTimer(ros::Duration(2.0), time_c_b_vel);
  print_param();
  

  
  int choice = 0;
  std::cout << "1 to go on , else to quit" << std::endl;
  std::cin >> choice;
  if (choice != 1) return 0;
  ros::spinOnce();
  rate.sleep();
  
  // 等待连接到飞控
  while (ros::ok() && !current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }
  //设置无人机的期望位置
 
  setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
  setpoint_raw.coordinate_frame = 1;
  setpoint_raw.position.x = 0;
  setpoint_raw.position.y = 0;
  setpoint_raw.position.z = ALTITUDE;
  setpoint_raw.yaw = 0;

  // send a few setpoints before starting
  for (int i = 100; ros::ok() && i > 0; --i)
  {
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();
  }
  std::cout<<"ok"<<std::endl;


  // 定义客户端变量，设置为offboard模式
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  // 定义客户端变量，请求无人机解锁
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  // 记录当前时间，并赋值给变量last_request
  ros::Time last_request = ros::Time::now();

  while (ros::ok())
  {
    if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(3.0)))
    {
      if(if_debug == 1)
      {
        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        {
          ROS_INFO("Offboard enabled");
        }
      }
      else
      {
        ROS_INFO("Waiting for OFFBOARD mode");
      }
      last_request = ros::Time::now();
    }
    else
    {
      if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0)))
      {
        if (arming_client.call(arm_cmd) && arm_cmd.response.success)
        {
          ROS_INFO("Vehicle armed");
        }
        last_request = ros::Time::now();
      }
    }
    // 当无人机到达起飞点高度后，悬停3秒后进入任务模式，提高视觉效果
    if (fabs(local_pos.pose.pose.position.z - ALTITUDE) < 0.2)
    {
      if (ros::Time::now() - last_request > ros::Duration(1.0))
      {
        mission_num = 1;
 	      last_request = ros::Time::now();
        break;
      }
    }

    mission_pos_cruise(0, 0, ALTITUDE, 0, err_max); 
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();
  }
  

  while (ros::ok())
  {
    ROS_WARN("mission_num = %d", mission_num);
    
    switch (mission_num)
    {
      // mission1: 起飞
      case 1:{
        if (mission_pos_cruise(0, 0, ALTITUDE, 0, err_max))
        {
          mission_num = 2;
          last_request = ros::Time::now();
        }
	    else if(ros::Time::now() - last_request >= ros::Duration(3.0))
        {
          mission_num = 2;
          last_request = ros::Time::now();
        }
        break;
      }

      //世界系前进
      case 2:
      {
        static bool initialized = false;
        static ros::Time timer_20;
        static bool is_10s_cycle_running = false;  // 标记10秒周期是否运行
        static point cached_temp_target;           // 新增：缓存生成的假点（固定不变）
        static bool temp_target_generated = false; // 新增：标记假点是否已生成

        // 初始化一次（含假点相关标记重置）
        if (!initialized)
        {
          last_request = ros::Time::now(); // 总超时计时起点
          timer_20 = ros::Time::now();     // 10秒周期计时起点
          initialized = true;
          is_10s_cycle_running = true;       // 启动10秒周期
          temp_target_generated = false;     // 初始化时重置假点标记
          cached_temp_target = {0.0f, 0.0f}; // 初始化假点缓存
        }

        // 180秒总超时：重置所有标记+退出
        if (ros::Time::now() - last_request > ros::Duration(180.0))
        {
          mission_num = 3;
          initialized = false;
          is_10s_cycle_running = false;
          temp_target_generated = false; // 超时后重置假点标记
          break;                         // 退出 switch
        }

        // 10秒周期内执行避障逻辑
        if (is_10s_cycle_running && (ros::Time::now() - timer_20 < ros::Duration(10.0)))
        {
          ROS_INFO("进入避障!!!");
          ROS_WARN("执行10秒周期内的避障逻辑!!!");

          // 场景1：已生成假点 → 持续执行该假点的避障任务（不重新计算）
          if (temp_target_generated)
          {
            ROS_WARN("持续执行假点避障，假点坐标：(%.2f, %.2f)", cached_temp_target.x, cached_temp_target.y);
            // 执行假点避障，判断是否到达假点
            if (collision_avoidance_mission(cached_temp_target.x, cached_temp_target.y, ALTITUDE, 0, err_max))
            {
              ROS_WARN("到达假点（脱离势能陷阱），返回原始目标点继续避障！");
              // 仅重置假点标记，不切换mission_num（核心修改）
              temp_target_generated = false;     // 重置假点标记，下次执行原始目标避障
              cached_temp_target = {0.0f, 0.0f}; // 清空假点缓存
              timer_20 = ros::Time::now();       // 重置10秒周期，重新开始判断
            }
          }
          // 场景2：未生成假点 → 先判断震荡状态
          else
          {
            if (stuck_detection(current_pos, current_vel))
            {
              // 震荡且未生成假点 → 仅计算一次假点并缓存
              CalcErr err;
              cached_temp_target = cal_temporary_waypoint(target, current_pos.back(), distance_c, angle_c, &err);
              temp_target_generated = true; // 标记假点已生成（后续不再重新计算）
              timer_20 = ros::Time::now();  // 震荡时重置10秒计时器
              ROS_WARN("检测到震荡，生成假点并固定：(%.2f, %.2f)", cached_temp_target.x, cached_temp_target.y);
              // 执行首次假点避障
              collision_avoidance_mission(cached_temp_target.x, cached_temp_target.y, ALTITUDE, 0, err_max);
            }
            // 未震荡 → 执行原始目标避障
            else
            {
              if (collision_avoidance_mission(target.x, target.y, ALTITUDE, 0, err_max))
              {
                mission_num = 3;
                ROS_INFO("原始目标!!!(%.2f, %.2f)", target.x, target.y);
                initialized = false;
                is_10s_cycle_running = false;
                break; // 退出case2
              }
            }
          }
          mavros_setpoint_pos_pub.publish(setpoint_raw);
        }
        // 10秒周期结束 → 仅重置周期标记（假点标记保留，若已生成则下次继续执行）
        else if (is_10s_cycle_running)
        {
          flag = 0;
          ROS_INFO("10秒周期结束，重置周期计时器（假点状态保留）!!!");
          timer_20 = ros::Time::now();
          is_10s_cycle_running = false;
        }
        // 10秒周期结束后重启 → 恢复周期执行（假点标记仍保留）
        else
        {
          ROS_INFO("重启10秒避障周期（假点状态保留）!!!");
          is_10s_cycle_running = true;
          timer_20 = ros::Time::now();
        }
        break;
      }

      //降落
      case 3:{
        // ========== 新增降落悬停变量 ==========
        static ros::Time land_hover_start;      // 降落悬停开始时间
        static bool is_land_hovering = false;   // 是否正在降落悬停
        static bool is_land_hover_done = false; // 降落悬停是否完成

        last_request = ros::Time::now();

        // 未到达降落点：执行精确定位降落
        if (!is_land_hover_done && !is_land_hovering)
        {
          if (precision_land())
          {
            ROS_WARN("到达降落点，开始悬停10秒！");
            is_land_hovering = true;             // 标记开始降落悬停
            land_hover_start = ros::Time::now(); // 记录悬停开始时间
          }
          else if (ros::Time::now() - last_request > ros::Duration(20.0))
          {
            ROS_WARN("降落超时，强制悬停10秒后结束任务！");
            is_land_hovering = true;
            land_hover_start = ros::Time::now();
          }
        }
        // 降落悬停逻辑
        else if (is_land_hovering)
        {
          // 悬停期间：保持降落点位置悬停
          mission_pos_cruise(local_pos.pose.pose.position.x,
                             local_pos.pose.pose.position.y,
                             local_pos.pose.pose.position.z, 0, err_max);
          mavros_setpoint_pos_pub.publish(setpoint_raw);
          ROS_INFO("降落点悬停中，剩余时长：%.1f秒",
                   HOVER_DURATION - (ros::Time::now() - land_hover_start).toSec());

          // 悬停满10秒：结束任务
          if (ros::Time::now() - land_hover_start > ros::Duration(HOVER_DURATION))
          {
            ROS_WARN("降落点悬停10秒完成，任务结束！");
            is_land_hovering = false;
            is_land_hover_done = true;
            mission_num = -1; // 任务结束
          }
        }
        break;
      }
    }
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();
    
    if(mission_num == -1) 
    {
      exit(0);
    }
  }
  return 0;
}
