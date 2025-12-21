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
    // 【修改1】赋值给全局变量，而非定义局部变量覆盖
    current_pos = Eigen::Vector2f(local_pos.pose.pose.position.x, local_pos.pose.pose.position.y);

    // 【修改2】存储Eigen::Vector2f格式的速度（替代原Vel结构体）
    tf::Vector3 body_vel(local_pos.twist.twist.linear.x, local_pos.twist.twist.linear.y, local_pos.twist.twist.linear.z);
    tf::Matrix3x3 rot_matrix(quat);
    tf::Vector3 world_vel = rot_matrix * body_vel;
    current_vel = Eigen::Vector2f(world_vel.x(), world_vel.y());

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

// 避障状态机枚举（静态，避免全局污染）
static enum class AvoidanceState {
    IDLE,           // 无避障，直接飞向目标
    DECELERATE,     // 碰撞锥激活，减速
    FLY_TO_TANGENT, // 飞向最优/融合切点
    ARC_FLIGHT,     // 沿弧形轨迹避障
    RETURN_TO_GOAL  // 脱离碰撞锥，回归目标
} avoid_state = AvoidanceState::IDLE;

// 避障静态变量（仅初始化一次）
static Eigen::Vector2f opt_tangent;     // 最优/融合切点
static Eigen::Vector2f arc_center;      // 弧形中心（障碍物扩增圆圆心）
static float arc_radius;                // 弧形半径（无人机到切点距离）
static ros::Time decelerate_start_time; // 减速开始时间
static ros::Time tangent_arrive_time;   // 到达切点开始计时

/**
 * @brief 计算碰撞锥相关参数（适配ObsRound结构体）
 */
std::tuple<float, float> calculateCollisionCone(
    const Eigen::Vector2f &UAV_pos,
    const Eigen::Vector2f &UAV_dir,
    const ObsRound &obs,
    float UAV_radius)
{
    Eigen::Vector2f r = obs.position - UAV_pos;
    float r_norm = r.norm();
    if (r_norm < 1e-3)
    {
        return {M_PI, M_PI}; // 距离过近，默认最大锥角
    }

    // 碰撞锥开度角：基于安全半径计算
    float cone_opening_angle = asin(std::min(1.0f, obs.safe_radius / r_norm));
    float dir_angle = acos(std::max(-1.0f, std::min(1.0f, UAV_dir.dot(r.normalized()))));

    return {cone_opening_angle, dir_angle};
}

/**
 * @brief 选择最优/融合切点（适配ObsRound，解决多障碍物冲突）
 */
Eigen::Vector2f selectOptimalTangent(
    const std::vector<ObsRound> &obs_rounds,
    const Eigen::Vector2f &UAV_pos,
    const Eigen::Vector2f &target)
{
    // 无障碍物：返回目标点
    if (obs_rounds.empty())
        return target;

    // 单障碍物：选到目标更近的切点
    if (obs_rounds.size() == 1)
    {
        float dist_left = (obs_rounds[0].left_point - target).norm();
        float dist_right = (obs_rounds[0].right_point - target).norm();
        return dist_left < dist_right ? obs_rounds[0].left_point : obs_rounds[0].right_point;
    }

    // 多障碍物：融合切点（距离加权平均，解决冲突）
    Eigen::Vector2f fuse_tangent = Eigen::Vector2f::Zero();
    float total_weight = 0.0f;
    for (const auto &obs : obs_rounds)
    {
        // 每个障碍物选最优切点
        float dist_left = (obs.left_point - target).norm();
        float dist_right = (obs.right_point - target).norm();
        Eigen::Vector2f obs_opt = dist_left < dist_right ? obs.left_point : obs.right_point;

        // 权重=1/无人机到障碍物的距离（近障碍权重高）
        float dist_uav_obs = (obs.position - UAV_pos).norm();
        float weight = dist_uav_obs < 1e-3 ? 1.0f : (1.0f / dist_uav_obs);

        fuse_tangent += obs_opt * weight;
        total_weight += weight;
    }
    return total_weight < 1e-3 ? target : (fuse_tangent / total_weight);
}

/**
 * @brief 筛选激活障碍物（碰撞锥重叠的障碍物）
 */
std::vector<ObsRound> getActiveObs(
    const std::vector<ObsRound> &obs_rounds,
    const Eigen::Vector2f &UAV_pos,
    const Eigen::Vector2f &target)
{
    std::vector<ObsRound> active_obs;
    Eigen::Vector2f UAV_dir = target - UAV_pos;
    if (UAV_dir.norm() < 1e-3)
        return active_obs;
    UAV_dir.normalize();

    for (const auto &obs : obs_rounds)
    {
        auto [cone_opening, dir_angle] = calculateCollisionCone(UAV_pos, UAV_dir, obs, UAV_radius);
        // 碰撞锥重叠（方向角≤开度角+5°安全裕度）→ 激活
        if (dir_angle <= cone_opening + 0.087f)
        { // 0.087rad=5°
            active_obs.push_back(obs);
        }
    }

    // 按距离排序（近→远），解决多障碍物优先级
    std::sort(active_obs.begin(), active_obs.end(),
              [&UAV_pos](const ObsRound &a, const ObsRound &b)
              {
                  return (a.position - UAV_pos).norm() < (b.position - UAV_pos).norm();
              });
    return active_obs;
}

/**
 * @brief 基于切点的圆锥避障位置计算（核心逻辑）
 */
Eigen::Vector2f coneAvoidanceByTangent(
    const Eigen::Vector2f &target,
    const Eigen::Vector2f &UAV_pos,
    const Eigen::Vector2f &UAV_vel)
{
    // 常量定义（复用原有参数）
    const float MAX_SPEED = 2.0f;
    const float MIN_HOVER_SPEED = 0.1f;
    const float err_max = 0.2f;      // 复用原有误差阈值
    const float ROTATE_SPEED = 0.1f; // 弧形旋转角速度（rad/s）

    // 步骤1：筛选激活障碍物
    std::vector<ObsRound> active_obs = getActiveObs(obs_rounds, UAV_pos, target);

    // 步骤2：状态机逻辑
    switch (avoid_state)
    {
    // 状态1：无避障，直接飞向目标
    case AvoidanceState::IDLE:
    {
        if (!active_obs.empty())
        {
            // 触发避障，进入减速状态
            avoid_state = AvoidanceState::DECELERATE;
            decelerate_start_time = ros::Time::now();
            opt_tangent = selectOptimalTangent(active_obs, UAV_pos, target);
            ROS_INFO("检测到激活障碍物(%d个)，进入减速状态，最优切点：(%.2f, %.2f)",
                     active_obs.size(), opt_tangent.x(), opt_tangent.y());
            // 第一步先减速
            Eigen::Vector2f dir = target - UAV_pos;
            dir.normalize();
            return UAV_pos + dir * MIN_HOVER_SPEED * 0.05f;
        }
        // 无避障，直接飞向目标
        Eigen::Vector2f dir = target - UAV_pos;
        if (dir.norm() > 1e-3)
            dir.normalize();
        return UAV_pos + dir * MAX_SPEED * 0.05f;
    }

    // 状态2：线性减速（2秒降至MIN_HOVER_SPEED）
    case AvoidanceState::DECELERATE:
    {
        float decel_duration = (ros::Time::now() - decelerate_start_time).toSec();
        float current_speed = std::max(MIN_HOVER_SPEED,
                                       MAX_SPEED * (1 - decel_duration / 2.0f));
        // 飞向最优切点
        Eigen::Vector2f dir = opt_tangent - UAV_pos;
        if (dir.norm() < 1e-3)
            dir = Eigen::Vector2f::UnitX();
        else
            dir.normalize();

        // 减速完成，进入飞向切点状态
        if (current_speed <= MIN_HOVER_SPEED + 1e-3)
        {
            avoid_state = AvoidanceState::FLY_TO_TANGENT;
            ROS_INFO("减速完成（当前速度：%.2f），飞向切点", current_speed);
        }
        return UAV_pos + dir * current_speed * 0.05f;
    }

    // 状态3：精准飞向切点
    case AvoidanceState::FLY_TO_TANGENT:
    {
        // 到达切点判定（误差<err_max）
        if ((opt_tangent - UAV_pos).norm() < err_max)
        {
            // 初始化弧形飞行参数
            avoid_state = AvoidanceState::ARC_FLIGHT;
            arc_center = active_obs[0].position; // 近障碍为弧形中心
            arc_radius = (opt_tangent - arc_center).norm();
            tangent_arrive_time = ros::Time::now();
            ROS_INFO("到达切点，进入弧形避障（中心：(%.2f, %.2f)，半径：%.2f）",
                     arc_center.x(), arc_center.y(), arc_radius);
            return opt_tangent;
        }
        // 未到达，继续飞向切点
        Eigen::Vector2f dir = opt_tangent - UAV_pos;
        dir.normalize();
        return UAV_pos + dir * MIN_HOVER_SPEED * 0.05f;
    }

    // 状态4：弧形避障（圆锥轨迹）
    case AvoidanceState::ARC_FLIGHT:
    {
        // 计算弧形下一个点（绕障碍中心旋转）
        float arc_duration = (ros::Time::now() - tangent_arrive_time).toSec();
        float rotate_angle = ROTATE_SPEED * arc_duration; // 累计旋转角度

        // 旋转矩阵：绕arc_center顺时针旋转（适配圆锥避障）
        Eigen::Rotation2D<float> rot(-rotate_angle);
        Eigen::Vector2f arc_point = arc_center + rot * (opt_tangent - arc_center);

        // 检测是否脱离碰撞锥
        Eigen::Vector2f UAV_dir = target - UAV_pos;
        UAV_dir.normalize();
        auto [cone_opening, dir_angle] = calculateCollisionCone(UAV_pos, UAV_dir, active_obs[0], UAV_radius);
        if (dir_angle > cone_opening + 0.087f)
        { // 脱离碰撞锥（+5°裕度）
            avoid_state = AvoidanceState::RETURN_TO_GOAL;
            ROS_INFO("脱离碰撞锥（夹角：%.1f°），回归目标", dir_angle * 180 / M_PI);
            return arc_point;
        }
        return arc_point;
    }

    // 状态5：回归目标（线性加速）
    case AvoidanceState::RETURN_TO_GOAL:
    {
        float return_duration = (ros::Time::now() - tangent_arrive_time).toSec();
        // 2秒线性加速至MAX_SPEED
        float current_speed = std::min(MAX_SPEED,
                                       MIN_HOVER_SPEED + (MAX_SPEED - MIN_HOVER_SPEED) * return_duration / 2.0f);
        // 飞向最终目标
        Eigen::Vector2f dir = target - UAV_pos;
        dir.normalize();

        // 加速完成，回到IDLE状态
        if (current_speed >= MAX_SPEED - 1e-3)
        {
            avoid_state = AvoidanceState::IDLE;
            ROS_INFO("加速完成，回到正常飞行状态");
        }
        return UAV_pos + dir * current_speed * 0.05f;
    }
    }

    // 兜底：返回当前位置
    return UAV_pos;
}
bool cone_avoidance_movement(float target_x, float target_y, float target_z,
                             float target_yaw, float UAV_radius,
                             float time_final, float err_max)
{
    // ================= 1. 初始化（首次调用） =================
    static bool is_init = false;
    static ros::Time start_time;
    if (!is_init)
    {
        start_time = ros::Time::now();
        is_init = true;
        avoid_state = AvoidanceState::IDLE; // 重置避障状态机
        ROS_INFO("圆锥避障任务启动，超时阈值：%.1f秒", time_final);
    }

    // ================= 2. 终止条件判断（保留原有逻辑） =================
    ros::Duration elapsed_time = ros::Time::now() - start_time;
    // 2.1 超时终止
    if (elapsed_time.toSec() >= time_final)
    {
        ROS_WARN("圆锥避障任务超时（已耗时%.1f秒），准备降落！", elapsed_time.toSec());
        is_init = false;
        return true;
    }
    // 2.2 到达目标点终止
    float abs_target_x = target_x + init_position_x_take_off;
    float abs_target_y = target_y + init_position_y_take_off;
    float dist_x = local_pos.pose.pose.position.x - abs_target_x;
    float dist_y = local_pos.pose.pose.position.y - abs_target_y;
    float dist_xy = hypotf(dist_x, dist_y);
    float dist_z = fabs(local_pos.pose.pose.position.z - (target_z + init_position_z_take_off));
    if (dist_xy < err_max && dist_z < err_max)
    {
        ROS_INFO("到达目标点（距离误差：%.3f米），准备降落！", dist_xy);
        is_init = false;
        return true;
    }

    // ================= 3. 黑箱函数调用：转换障碍物为扩增圆+切点 =================
    extern std::vector<Obstacle> obstacles; // 原有障碍物列表
    transObs(obstacles);                    // 调用黑箱函数，生成obs_rounds（含切点/安全半径）

    // ================= 4. 输入参数准备 =================
    Eigen::Vector2f UAV_pos = current_pos;              // 无人机当前位置
    Eigen::Vector2f UAV_vel = current_vel;              // 无人机当前速度
    Eigen::Vector2f target(abs_target_x, abs_target_y); // 绝对目标点

    // ================= 5. 调用圆锥避障位置计算函数 =================
    Eigen::Vector2f next_pos = coneAvoidanceByTangent(target, UAV_pos, UAV_vel);

    // ================= 6. 设置位置控制指令（替换原有速度控制） =================
    // type_mask：启用位置控制，关闭速度/加速度/yaw_rate等冗余控制
    setpoint_raw.type_mask = 8 + 16 + 32 + 64 + 128 + 256 + 1024 + 2048;
    setpoint_raw.coordinate_frame = 1;                             // 局部NED坐标系
    setpoint_raw.position.x = next_pos.x();                        // 避障计算的下一个位置X
    setpoint_raw.position.y = next_pos.y();                        // 避障计算的下一个位置Y
    setpoint_raw.position.z = target_z + init_position_z_take_off; // 固定Z高度
    setpoint_raw.yaw = target_yaw;                                 // 固定偏航角

    // ================= 7. 日志输出（增强避障状态） =================
    std::string state_str;
    switch (avoid_state)
    {
    case AvoidanceState::IDLE:
        state_str = "正常飞行";
        break;
    case AvoidanceState::DECELERATE:
        state_str = "减速";
        break;
    case AvoidanceState::FLY_TO_TANGENT:
        state_str = "飞向切点";
        break;
    case AvoidanceState::ARC_FLIGHT:
        state_str = "弧形避障";
        break;
    case AvoidanceState::RETURN_TO_GOAL:
        state_str = "回归目标";
        break;
    }
    ROS_INFO(
        "状态：%s | 当前位置：(%.2f, %.2f) | 目标点：(%.2f, %.2f) | 下一位置：(%.2f, %.2f) | 剩余时间：%.1f秒",
        state_str.c_str(),
        UAV_pos.x(), UAV_pos.y(),
        target.x(), target.y(),
        next_pos.x(), next_pos.y(),
        time_final - elapsed_time.toSec());

    // ================= 8. 未满足终止条件，继续执行 =================
    return false;
}