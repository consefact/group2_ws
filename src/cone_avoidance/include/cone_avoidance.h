#include <string>
#include <vector>
#include"new_detect_obs.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/PositionTarget.h>
#include <cmath>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandLong.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <livox_ros_driver/CustomMsg.h>
#include<eigen3/Eigen/Dense>

using namespace std;

#define ALTITUDE 0.7f

mavros_msgs::PositionTarget setpoint_raw;

Eigen::Vector2f current_pos; // 无人机历史位置（二维）
Eigen::Vector2f current_vel; // 无人机历史速度（二维）
Eigen::Vector2f current_pos; // 无人机历史位置（二维）
Eigen::Vector2f current_vel; // 无人机历史速度（二维）

/************************************************************************
函数 1：无人机状态回调函数
*************************************************************************/
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg);
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

/************************************************************************
函数 2：回调函数接收无人机的里程计信息
从里程计信息中提取无人机的位置信息和姿态信息
*************************************************************************/
tf::Quaternion quat;
nav_msgs::Odometry local_pos;
double roll, pitch, yaw;
float init_position_x_take_off = 0;
float init_position_y_take_off = 0;
float init_position_z_take_off = 0;
float init_yaw_take_off = 0;
bool flag_init_position = false;
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg);
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    local_pos = *msg;
    tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    // 【修改1】存储Eigen::Vector2f格式的位置（替代原point结构体）
    Eigen::Vector2f  current_pos(local_pos.pose.pose.position.x, local_pos.pose.pose.position.y);

    // 【修改2】存储Eigen::Vector2f格式的速度（替代原Vel结构体）
    tf::Vector3 body_vel(local_pos.twist.twist.linear.x, local_pos.twist.twist.linear.y, local_pos.twist.twist.linear.z);
    tf::Matrix3x3 rot_matrix(quat);
    tf::Vector3 world_vel = rot_matrix * body_vel;
    Eigen::Vector2f current_vel(world_vel.x(), world_vel.y());

    if (flag_init_position == false && (local_pos.pose.pose.position.z > 0.1)) // 优化初始化阈值
    {
        init_position_x_take_off = local_pos.pose.pose.position.x;
        init_position_y_take_off = local_pos.pose.pose.position.y;
        init_position_z_take_off = local_pos.pose.pose.position.z;
        init_yaw_take_off = yaw;
        flag_init_position = true;
    }
}

/************************************************************************
函数 3: 无人机位置控制
控制无人机飞向（x, y, z）位置，target_yaw为目标航向角，error_max为允许的误差范围
进入函数后开始控制无人机飞向目标点，返回值为bool型，表示是否到达目标点
*************************************************************************/
float mission_pos_cruise_last_position_x = 0;
float mission_pos_cruise_last_position_y = 0;
// ========== 第七处修改：超时阈值改为可配置变量，设置默认初值 ==========
float mission_cruise_timeout = 180.0f;    // 普通巡航超时阈值默认值（秒）
ros::Time mission_cruise_start_time;      // 巡航任务开始时间
bool mission_cruise_timeout_flag = false; // 巡航超时标志
// ========== 修改结束 ==========
bool mission_pos_cruise_flag = false;
bool mission_pos_cruise(float x, float y, float z, float target_yaw, float error_max);
bool mission_pos_cruise(float x, float y, float z, float target_yaw, float error_max)
{
    if (mission_pos_cruise_flag == false)
    {
        mission_pos_cruise_last_position_x = local_pos.pose.pose.position.x;
        mission_pos_cruise_last_position_y = local_pos.pose.pose.position.y;
        mission_pos_cruise_flag = true;
        mission_cruise_start_time = ros::Time::now(); // 第七处修改：记录启动时间
        mission_cruise_timeout_flag = false;          // 第七处修改：重置超时标志
    }
    // ========== 第七处修改：巡航超时判断逻辑 ==========
    ros::Duration elapsed_time = ros::Time::now() - mission_cruise_start_time;
    if (elapsed_time.toSec() > mission_cruise_timeout && !mission_cruise_timeout_flag)
    {
        ROS_WARN("[巡航超时] 已耗时%.1f秒（阈值%.1f秒），强制切换下一个任务！", elapsed_time.toSec(), mission_cruise_timeout);
        mission_cruise_timeout_flag = true;
        mission_pos_cruise_flag = false; // 重置任务标志
        return true;                     // 返回true表示任务完成（超时切换）
    }
    // ========== 第七处修改==========
    setpoint_raw.type_mask = /*1 + 2 + 4 */ +8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048;
    setpoint_raw.coordinate_frame = 1;
    setpoint_raw.position.x = x + init_position_x_take_off;
    setpoint_raw.position.y = y + init_position_y_take_off;
    setpoint_raw.position.z = z + init_position_z_take_off;
    setpoint_raw.yaw = target_yaw;
    ROS_INFO("now (%.2f,%.2f,%.2f,%.2f) to ( %.2f, %.2f, %.2f, %.2f)", local_pos.pose.pose.position.x, local_pos.pose.pose.position.y, local_pos.pose.pose.position.z, target_yaw * 180.0 / M_PI, x + init_position_x_take_off, y + init_position_y_take_off, z + init_position_z_take_off, target_yaw * 180.0 / M_PI);
    if (fabs(local_pos.pose.pose.position.x - x - init_position_x_take_off) < error_max && fabs(local_pos.pose.pose.position.y - y - init_position_y_take_off) < error_max && fabs(local_pos.pose.pose.position.z - z - init_position_z_take_off) < error_max && fabs(yaw - target_yaw) < 0.1)
    {
        ROS_INFO("到达目标点，巡航点任务完成");
        mission_cruise_timeout_flag = false; // 第七处修改：重置超时标志
        mission_pos_cruise_flag = false;
        return true;
    }
    return false;
}

/************************************************************************
函数 4:降落
无人机当前位置作为降落点，缓慢下降至地面
返回值为bool型，表示是否降落完成
*************************************************************************/
float precision_land_init_position_x = 0;
float precision_land_init_position_y = 0;
bool precision_land_init_position_flag = false;
ros::Time precision_land_last_time;
bool precision_land();
bool precision_land()
{
    if (!precision_land_init_position_flag)
    {
        precision_land_init_position_x = local_pos.pose.pose.position.x;
        precision_land_init_position_y = local_pos.pose.pose.position.y;
        precision_land_last_time = ros::Time::now();
        precision_land_init_position_flag = true;
    }
    setpoint_raw.position.x = precision_land_init_position_x;
    setpoint_raw.position.y = precision_land_init_position_y;
    setpoint_raw.position.z = -0.15;
    setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
    setpoint_raw.coordinate_frame = 1;
    if (ros::Time::now() - precision_land_last_time > ros::Duration(5.0))
    {
        ROS_INFO("Precision landing complete.");
        precision_land_init_position_flag = false; // Reset for next landing
        return true;
    }
    return false;
}
/************************************************************************
函数 :计算速度
*************************************************************************/

/*
 * ===================== 参数定义 =====================
 */

// CBF 参数
float ALPHA = 2.0;
float MAX_SPEED = 2.0;
float OBS_EPS = 0.1;
float KV = 0.2;
float KN = 0.1;
float W_goal = 0.8;
float W_free = 0.2;

/**
 * @brief 基于 Control Barrier Function (CBF) 的一阶速度避障控制器
 *
 * 【函数功能】
 * 本函数为无人机（或移动机器人）生成一个“安全速度指令” u，
 * 在尽量跟随目标点方向运动的同时，保证与所有障碍物保持安全距离。
 *
 * 控制模型假设：
 *   位置动力学： p_dot = u
 *
 * 核心思想：
 * 1. 先生成一个指向目标的名义速度 u_des
 * 2. 识别对当前运动构成威胁的“激活障碍物”
 * 3. 构造一个参考速度 u_ref（目标驱动 + 避障启发）
 * 4. 对每个激活障碍物施加 CBF 线性安全约束
 * 5. 若违反约束，通过投影方式修正速度
 * 6. 输出满足速度上限的安全控制量
 *
 * @param target     目标点位置
 * @param UAV_pos    UAV 当前位姿
 * @param UAV_vel    UAV 当前速度（用于障碍物激活判断）
 * @param obstacles  圆形障碍物列表
 * @param UAV_radius UAV 自身等效半径
 *
 * @return Eigen::Vector2f  满足 CBF 安全约束的速度指令
 */
Eigen::Vector2f applyCBF(
    const Eigen::Vector2f &target,
    const Eigen::Vector2f &UAV_pos,
    const Eigen::Vector2f &UAV_vel,
    const std::vector<Obstacle> &obstacles,
    float UAV_radius)
{
    /* ================= 1. 名义控制：指向目标 ================= */
    Eigen::Vector2f u_des = target - UAV_pos;
    if (u_des.norm() > 1e-6f)
        u_des.normalize();
    Eigen::Vector2f u_ref = u_des * MAX_SPEED;

    /* ================= 2. 人工势场避障引导 ================= */
    Eigen::Vector2f free_dir = Eigen::Vector2f::Zero();
    for (const auto &obs : obstacles)
    {
        Eigen::Vector2f r = UAV_pos - obs.position;
        float dist = r.norm();
        float d_safe = UAV_radius + obs.radius;

        // 在影响范围内才产生斥力
        if (dist < d_safe + 5.0f) {
            free_dir += r / (dist*dist + 1e-3f);
        }
    }
    if (free_dir.norm() > 1e-3f) {
        free_dir.normalize();
        // 融合目标方向和避障方向
        u_ref = W_goal * u_ref + W_free * free_dir * u_ref.norm();
    }

    /* ================= 3. CBF 分析 & 解析投影修正 ================= */
    const float gamma = ALPHA;
    for (const auto &obs : obstacles)
    {
        Eigen::Vector2f r = UAV_pos - obs.position;
        float dist_sq = r.squaredNorm();
        float d_safe = UAV_radius + obs.radius;
        float safe_sq = d_safe*d_safe;

        // 只处理影响距离内
        if (dist_sq < (safe_sq + 10.0f*d_safe))
        {
            // 控制障碍函数
            float h = dist_sq - safe_sq;

            // 阶段一：梯度方向
            Eigen::Vector2f grad_h = 2.0f * r;

            // 违背 CBF 不等式量:  ∇h·u + γ h
            float vio = grad_h.dot(u_ref) + gamma * h;

            // 如果违背，则修正
            if (vio < 0.0f)
            {
                float denom = grad_h.squaredNorm() + 1e-6f;
                Eigen::Vector2f corr = (vio / denom) * grad_h;
                u_ref -= corr;
            }

            // 阶段二：如果非常接近安全边界，强制减小朝障碍速度
            float dist = std::sqrt(dist_sq);
            if (dist < d_safe + 0.5f*d_safe)
            {
                // 投影剔除条朝障碍的速度分量
                Eigen::Vector2f along = r.normalized();
                float v_along = u_ref.dot(along);
                if (v_along < 0.0f) {
                    u_ref -= v_along * along;
                }
            }
        }
    }

    /* ================= 4. 速度限幅 ================= */
    if (u_ref.norm() > MAX_SPEED)
        u_ref = u_ref.normalized() * MAX_SPEED;

    return u_ref;
}


/************************************************************************
函数 5: 圆锥避障前进函数（case2调用，每帧执行）
功能：集成CBF速度控制器，实现圆锥避障并向目标点移动
参数：
  - target_x/y: 目标点xy坐标（局部系，相对起飞点）
  - target_z: 目标点z高度（绝对高度）
  - target_yaw: 目标偏航角（弧度）
  - UAV_radius: 无人机自身等效半径（米）
  - time_final: 超时时间（秒）
  - err_max: 到达目标点的距离误差阈值（米）
返回值：
  - true: 超时/到达目标点，触发任务切换
  - false: 未完成，继续避障移动
*************************************************************************/
bool cone_avoidance_movement(float target_x, float target_y, float target_z,
                             float target_yaw, float UAV_radius,
                             float time_final, float err_max)
{
    // ================= 1. 初始化（首次调用） =================
    static bool is_init = false; // 首次调用标记
    static ros::Time start_time; // 计时器初始时间
    if (!is_init)
    {
        start_time = ros::Time::now(); // 首次调用初始化计时器
        is_init = true;
        ROS_INFO("圆锥避障任务启动，超时阈值：%.1f秒", time_final);
    }

    // ================= 2. 终止条件判断 =================
    // 2.1 计时到达阈值
    ros::Duration elapsed_time = ros::Time::now() - start_time;
    if (elapsed_time.toSec() >= time_final)
    {
        ROS_WARN("圆锥避障任务超时（已耗时%.1f秒），准备降落！", elapsed_time.toSec());
        is_init = false; // 重置初始化标记
        return true;
    }

    // 2.2 到达目标点（距离误差满足要求）
    float abs_target_x = target_x + init_position_x_take_off;
    float abs_target_y = target_y + init_position_y_take_off;
    float dist_x = local_pos.pose.pose.position.x - abs_target_x;
    float dist_y = local_pos.pose.pose.position.y - abs_target_y;
    float dist_xy = hypotf(dist_x, dist_y);
    if (dist_xy < err_max && fabs(local_pos.pose.pose.position.z - (target_z + init_position_z_take_off)) < err_max)
    {
        ROS_INFO("到达目标点（距离误差：%.3f米），准备降落！", dist_xy);
        is_init = false; // 重置初始化标记
        return true;
    }

    // ================= 3. 获取输入参数 =================
    // 3.1 无人机当前位置（二维，Eigen格式）
    Eigen::Vector2f UAV_pos = Eigen::Vector2f::Zero();
    if (!current_pos.iszero())
    {
        UAV_vel = current_pos; // 直接取Eigen::Vector2f（无需转换）
    }

    // 3.2 无人机当前速度（二维，Eigen格式，取最新值）
    Eigen::Vector2f UAV_vel = current_vel;

    // 3.3 目标点（二维，绝对坐标，Eigen格式）
    Eigen::Vector2f target(abs_target_x, abs_target_y);

    // 3.4 障碍物列表（通过Livox回调获取点列，转换为Obstacle）
    

    // 3.5 无人机自身半径（传入参数）

    // ================= 4. 调用CBF速度计算函数 =================
    Eigen::Vector2f safe_vel = applyCBF(target, UAV_pos, UAV_vel, obstacles, UAV_radius);

    // ================= 5. 设置速度控制指令 =================
    // type_mask：关闭位置控制(1+2+4)，启用速度控制，关闭其他冗余控制
    setpoint_raw.type_mask = 1 + 2 + 4 + 64 + 128 + 256 + 512 + 1024 + 2048;
    setpoint_raw.coordinate_frame = 1;                             // 局部NED坐标系
    setpoint_raw.velocity.x = safe_vel.x();                        // CBF输出x速度
    setpoint_raw.velocity.y = safe_vel.y();                        // CBF输出y速度
    setpoint_raw.position.z = target_z + init_position_z_take_off; // 固定z高度
    setpoint_raw.yaw = target_yaw;                                 // 固定偏航角

    // ================= 6. 日志输出 =================
    ROS_INFO(
        "当前位置：(%.2f, %.2f) | 目标点：(%.2f, %.2f) | 控制速度：(%.2f, %.2f) | 剩余时间：%.1f秒",
        UAV_pos.x(), UAV_pos.y(),
        target.x(), target.y(),
        safe_vel.x(), safe_vel.y(),
        time_final - elapsed_time.toSec());

    // ================= 7. 未满足终止条件，继续执行 =================
    return false;
}