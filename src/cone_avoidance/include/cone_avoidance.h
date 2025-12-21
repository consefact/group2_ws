#include <string>
#include <vector>
#include"new_detect_obs.h"
#include "cul_obs_round.h"
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

// 避障状态机枚举（仅移除ARC_FLIGHT，保留原有命名）
enum class AvoidanceState
{
    IDLE,           // 无避障，直接飞向目标
    DECELERATE,     // 碰撞锥激活，减速
    FLY_TO_TANGENT, // 飞向最优/融合切点
    RETURN_TO_GOAL  // 脱离碰撞锥，回归目标
};

// 保留所有静态变量（避免崩溃）
static AvoidanceState avoid_state = AvoidanceState::IDLE;
static Eigen::Vector2f opt_tangent;     // 最优/融合切点
static Eigen::Vector2f arc_center;      // 保留（仅注释，不删除，避免崩溃）
static float arc_radius;                // 保留（仅注释，不删除，避免崩溃）
static ros::Time decelerate_start_time; // 减速开始时间
static ros::Time tangent_arrive_time;   // 到达切点开始计时

/**
 * @brief 计算碰撞锥相关参数（完全保留原有逻辑）
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
} /**
   * @brief 选择最优切点（核心修改：选y坐标相对于无人机最远的切点，多障碍直接选全局最远）
   */
Eigen::Vector2f selectOptimalTangent(
    const std::vector<ObsRound> &obs_rounds,
    const Eigen::Vector2f &UAV_pos,
    const Eigen::Vector2f &target)
{
    // 无障碍物：返回目标点
    if (obs_rounds.empty())
        return target;

    // 提取无人机当前y坐标（核心参考值）
    float uav_y = UAV_pos.y();

    // 单障碍物：选该障碍左/右切点中y坐标最远的（|y_tangent - uav_y|最大）
    if (obs_rounds.size() == 1)
    {
        const auto &obs = obs_rounds[0];
        // 计算左/右切点y与无人机y的绝对值差
        float left_y_diff = fabs(obs.left_point.y() - uav_y);
        float right_y_diff = fabs(obs.right_point.y() - uav_y);

        // 选y差值更大的切点
        Eigen::Vector2f selected_tangent = (left_y_diff > right_y_diff) ? obs.left_point : obs.right_point;

        ROS_INFO("单障碍切点选择：左切点y=%.2f（差=%.2f），右切点y=%.2f（差=%.2f），选y最远切点=(%.2f,%.2f)",
                 obs.left_point.y(), left_y_diff, obs.right_point.y(), right_y_diff,
                 selected_tangent.x(), selected_tangent.y());
        return selected_tangent;
    }

    // 多障碍物：直接选全局y坐标最远的切点（|y_tangent - uav_y|最大）
    Eigen::Vector2f farthest_tangent; // 全局y最远切点
    float max_y_diff = -1.0f;         // 最大y差值（初始为负，确保首次赋值）

    for (const auto &obs : obs_rounds)
    {
        // 步骤1：给当前障碍选左/右切点中y坐标最远的
        float left_y_diff = fabs(obs.left_point.y() - uav_y);
        float right_y_diff = fabs(obs.right_point.y() - uav_y);
        Eigen::Vector2f obs_farthest = (left_y_diff > right_y_diff) ? obs.left_point : obs.right_point;
        float obs_y_diff = (left_y_diff > right_y_diff) ? left_y_diff : right_y_diff;

        // 步骤2：对比当前障碍的y最远切点与全局，保留更远的
        if (obs_y_diff > max_y_diff)
        {
            max_y_diff = obs_y_diff;
            farthest_tangent = obs_farthest;
        }

        // 日志：输出当前障碍的切点y信息
        ROS_INFO("障碍切点：左(%.2f, %.2f)y差=%.2f | 右(%.2f, %.2f)y差=%.2f | 选该障碍y最远切点=(%.2f,%.2f)",
                 obs.left_point.x(), obs.left_point.y(), left_y_diff,
                 obs.right_point.x(), obs.right_point.y(), right_y_diff,
                 obs_farthest.x(), obs_farthest.y());
    }

    // 边界处理：若所有y差值为0（极端情况），返回目标点
    if (max_y_diff < 1e-3)
    {
        ROS_WARN("所有切点y坐标与无人机几乎重合，返回目标点");
        return target;
    }

    ROS_INFO("多障碍全局y最远切点（y差=%.2f）：(%.2f,%.2f)", max_y_diff, farthest_tangent.x(), farthest_tangent.y());
    return farthest_tangent;
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
 * @brief 基于切点的圆锥避障位置计算（核心逻辑：仅简化状态机，保留static）
 */
Eigen::Vector2f coneAvoidanceByTangent(
    const Eigen::Vector2f &target,
    const Eigen::Vector2f &UAV_pos,
    const Eigen::Vector2f &UAV_vel)
{
    // 常量定义（完全保留原有参数）
    const float MAX_SPEED = 2.0f;
    const float MIN_HOVER_SPEED = 0.1f;
    const float err_max = 0.2f;      // 复用原有误差阈值
    const float ROTATE_SPEED = 0.1f; // 保留（不使用，避免删除导致崩溃）

    // 步骤1：筛选激活障碍物（保留static，避免崩溃）
    static std::vector<ObsRound> active_obs = getActiveObs(obs_rounds, UAV_pos, target);
    // 关键调整：每次都更新active_obs（而非仅首次）
    active_obs = getActiveObs(obs_rounds, UAV_pos, target);

    // 步骤2：状态机逻辑（仅移除ARC_FLIGHT，调整FLY_TO_TANGENT后直接回归目标）
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
            // 关键调整：每次触发避障都更新切点（而非仅首次）
            opt_tangent = selectOptimalTangent(active_obs, UAV_pos, target);
            ROS_INFO("检测到激活障碍物(%zu个)，进入减速状态，最优切点：(%.2f, %.2f)",
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

    // 状态3：精准飞向切点（到达后直接回归目标，移除弧形避障）
    case AvoidanceState::FLY_TO_TANGENT:
    {
        // 到达切点判定（误差<err_max）
        if ((opt_tangent - UAV_pos).norm() < err_max)
        {
            // 调整：到达切点后直接进入回归目标状态，移除弧形避障
            avoid_state = AvoidanceState::RETURN_TO_GOAL;
            tangent_arrive_time = ros::Time::now();
            ROS_INFO("到达切点(%.2f, %.2f)，直接回归目标",
                     opt_tangent.x(), opt_tangent.y());
            return opt_tangent;
        }
        // 未到达，继续飞向切点
        Eigen::Vector2f dir = opt_tangent - UAV_pos;
        dir.normalize();
        return UAV_pos + dir * MAX_SPEED * 0.5f;
    }

    // 状态5：回归目标（线性加速，保留原有逻辑）
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
    static int log_count = 0;
    if (if_debug == 1 && ++log_count % 10 == 0)
    {
        for (int i = 0; i < obs_rounds.size(); i++)
        {

            ROS_INFO("障碍物%d:(%.2f,%.2f),r=%.2f,sr=%.2f,l(%.2f,%.2f),r(%.2f,%.2f)", i, obs_rounds[i].position.x(), obs_rounds[i].position.y(), obs_rounds[i].radius, obs_rounds[i].safe_radius, obs_rounds[i].left_point.x(), obs_rounds[i].left_point.y(), obs_rounds[i].right_point.x(), obs_rounds[i].right_point.y());
        }
    }

    // ================= 4. 输入参数准备 =================
    Eigen::Vector2f UAV_pos = current_pos;              // 无人机当前位置
    Eigen::Vector2f UAV_vel = current_vel;              // 无人机当前速度
    Eigen::Vector2f target(abs_target_x, abs_target_y); // 绝对目标点

    // ================= 5. 调用圆锥避障位置计算函数 =================
    Eigen::Vector2f next_pos = coneAvoidanceByTangent(target, UAV_pos, UAV_vel);

    // ================= 6. 设置位置控制指令（保留原有逻辑，仅标注建议） =================
    // 【优化建议】type_mask=8+16+32+64+128+256+1024 更合理（移除2048，避免位置指令失效）
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