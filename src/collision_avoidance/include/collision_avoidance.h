#include <string>
#include <vector>
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

using namespace std;

#define ALTITUDE 0.7f

mavros_msgs::PositionTarget setpoint_raw;

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
typedef struct point
{
    float x;
    float y;
} point;
point target;

/* 线段结构体（segment） */
typedef struct segment
{
    point p1;
    point p2; // 两点确定一条直线
} segment;
struct Vel
{
    float x;
    float y;
};
std::vector<point> current_pos;
std::vector<Vel> current_vel;

// 错误码定义
typedef enum
{
    CALC_SUCCESS = 0,
    CALC_DIV_ZERO = 1,     // 除以零错误
    CALC_INVALID_PARAM = 2 // 参数非法
} CalcErr;
point getCross(segment seg, point point_p, CalcErr *err);
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg);
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    local_pos = *msg;
    tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    current_pos.push_back(point{local_pos.pose.pose.position.x, local_pos.pose.pose.position.y});
    tf::Vector3 body_vel(local_pos.twist.twist.linear.x, local_pos.twist.twist.linear.y, local_pos.twist.twist.linear.z);
    tf::Matrix3x3 rot_matrix(quat);
    tf::Vector3 world_vel = rot_matrix * body_vel;
    current_vel.push_back(Vel{world_vel.x(), world_vel.y()});

    if (flag_init_position == false && (local_pos.pose.pose.position.z != 0))
    {
        init_position_x_take_off = local_pos.pose.pose.position.x;
        init_position_y_take_off = local_pos.pose.pose.position.y;
        init_position_z_take_off = local_pos.pose.pose.position.z;
        init_yaw_take_off = yaw;
        flag_init_position = true;
    }
}
void time_c_b_pos(const ros::TimerEvent &event); // 每隔5秒删除一次储存的位置
void time_c_b_pos(const ros::TimerEvent &event)
{
    current_pos.clear();
}
void time_c_b_vel(const ros::TimerEvent &event); // 每隔5秒删除一次储存的速度
void time_c_b_vel(const ros::TimerEvent &event)
{
    current_vel.clear();
}
/************************************************************************
函数 3: 无人机位置控制
控制无人机飞向（x, y, z）位置，target_yaw为目标航向角，error_max为允许的误差范围
进入函数后开始控制无人机飞向目标点，返回值为bool型，表示是否到达目标点
*************************************************************************/
float mission_pos_cruise_last_position_x = 0;
float mission_pos_cruise_last_position_y = 0;
// ========== 第七处修改：超时阈值改为可配置变量，设置默认初值 ==========
float mission_cruise_timeout = 180.0f;     // 普通巡航超时阈值默认值（秒）
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
函数 5：cal_min_distance
计算激光雷达数据中的最小距离及其对应的角度索引
返回值：无
*************************************************************************/
float distance_c;
int angle_c;

double zero_plane_height = 0.0; // 0度平面高度
double height_threshold = 0.05; // 高度阈值
double min_range = 0.1;         // 最小检测距离
double max_range = 30.0;        // 最大检测距离
int num_bins = 360;             // 角度分bin数量

// 用于存储每个角度bin的最小距离
std::vector<float> distance_bins;
std::vector<int> count_bins;
void cal_min_distance()
{
    distance_c = distance_bins[min_range];
    angle_c = 0;
    for (int i = 0; i <= 359; i++)
    {
        if (distance_bins[i] < distance_c)
        {
            distance_c = distance_bins[i];
            angle_c = i;
        }
    }
    ROS_WARN("Minimum Distance: %.2f m at Angle: %d deg", distance_c, angle_c);
}

/************************************************************************
函数 6:lidar_cb
点云回调函数，处理Livox雷达的点云数据，实现360度障碍物检测
/livox/lidar example:

*************************************************************************/

void livox_custom_cb(const livox_ros_driver::CustomMsg::ConstPtr &livox_msg)
{
    // 初始化bins
    // ROS_INFO("Received Livox point cloud with %d points", livox_msg->point_num);
    distance_bins.assign(num_bins, max_range);
    count_bins.assign(num_bins, 0);

    int total_points = livox_msg->point_num;
    int plane_points = 0;

    // 遍历Livox自定义消息中的点
    for (int i = 0; i < total_points; i++)
    {
        const livox_ros_driver::CustomPoint &point = livox_msg->points[i];

        float x = point.x;
        float y = point.y;
        float z = point.z;

        // 筛选0度平面附近的点
        if (fabs(z - zero_plane_height) > height_threshold)
        {
            continue;
        }

        plane_points++;

        // 计算距离和角度
        float distance = sqrt(x * x + y * y);
        float angle = atan2(y, x); // 弧度

        // 转换为角度并映射到0-359
        int angle_bin = static_cast<int>((angle * 180.0 / M_PI));

        // 转换为0-359范围
        if (angle_bin < 0)
            angle_bin += 360;
        if (angle_bin >= 360)
            angle_bin -= 360;

        // 确保在有效范围内
        if (angle_bin >= 0 && angle_bin < num_bins)
        {
            // 只保留每个角度bin的最小距离
            if (distance >= min_range && distance <= max_range &&
                !std::isinf(distance) && !std::isnan(distance))
            {
                if (distance < distance_bins[angle_bin])
                {
                    distance_bins[angle_bin] = distance;
                }
                count_bins[angle_bin]++;
            }
        }
    }
    for (int i = 0; i < num_bins; i++)
    {
        if (distance_bins[i] == 0)
        {
            distance_bins[i] = max_range; // 如果该bin没有点，则设为最大距离
        }
        // ROS_INFO("Angle Bin %d: Min Distance = %.2f m, Point Count = %d", i, distance_bins[i], count_bins[i]);
    }
    cal_min_distance();
}

/************************************************************************
函数 7: satfunc
数据饱和函数，限制数据在±Max范围内
*************************************************************************/
float satfunc(float data, float Max)
{
    if (abs(data) > Max)
        return (data > 0) ? Max : -Max;
    else
        return data;
}

/************************************************************************
函数 8: collision_avoidance 避障函数
根据激光雷达数据计算避障速度，并与追踪速度叠加，得到最终速度指令
输入参数：目标位置target_x, target_y
返回值：true/false表示是否到达目标点
*************************************************************************/
float R_outside, R_inside;      // 安全半径 [避障算法相关参数]
float p_R;                      // 大圈比例参数
float p_r;                      // 小圈比例参数
float distance_cx, distance_cy; // 最近障碍物距离XY
float vel_collision[2];         // 躲避障碍部分速度
float vel_collision_max;        // 躲避障碍部分速度限幅
float p_xy;                     // 追踪部分位置环P
float vel_track[2];             // 追踪部分速度
float vel_track_max;            // 追踪部分速度限幅

float vel_sp_body[2];                    // 总速度
float vel_sp_ENU[2];                     // ENU下的总速度
float vel_sp_max;                        // 总速度限幅
std_msgs::Bool flag_collision_avoidance; // 是否进入避障模式标志位
// ========== 第七次修改：避障巡航超时阈值==========
float collision_cruise_timeout = 25.0f;     // 避障巡航超时阈值默认值（秒）
ros::Time collision_cruise_start_time;      // 避障巡航开始时间
bool collision_cruise_flag = false;         // 避障巡航初始化标志
bool collision_cruise_timeout_flag = false; // 避障巡航超时标志
ros::Time last_request;
// ========== 修改结束 ==========

void rotation_yaw(float yaw_angle, float input[2], float output[2])
{
    output[0] = input[0] * cos(yaw_angle) - input[1] * sin(yaw_angle);
    output[1] = input[0] * sin(yaw_angle) + input[1] * cos(yaw_angle);
}

bool collision_avoidance_mission(float target_x, float target_y, float target_z, float target_yaw, float err_max)
{
    // ========== 第七次：避障巡航首次进入初始化计时 ==========
    if (!collision_cruise_flag)
    {
        collision_cruise_start_time = ros::Time::now();
        collision_cruise_timeout_flag = false;
        collision_cruise_flag = true;
        ROS_INFO("[避障巡航] 任务启动，超时阈值%.1f秒", collision_cruise_timeout);
    }
    // ========== 第七次：避障巡航超时判断逻辑 ==========
    ros::Duration elapsed_time = ros::Time::now() - collision_cruise_start_time;
    if (elapsed_time.toSec() > collision_cruise_timeout && !collision_cruise_timeout_flag)
    {
        ROS_WARN("[避障巡航超时] 已耗时%.1f秒（阈值%.1f秒），强制切换下一个任务！", elapsed_time.toSec(), collision_cruise_timeout);
        collision_cruise_timeout_flag = true;
        collision_cruise_flag = false; // 重置任务标志
        return true;                   // 返回true表示任务完成（超时切换）
    }
    // ========== 新增结束 ==========
    // 2. 根据最小距离判断：是否启用避障策略
    if (distance_c >= R_outside)
    {
        flag_collision_avoidance.data = false;
    }
    else
    {
        flag_collision_avoidance.data = true;
    }

    // 3. 计算追踪速度,shijie
    if(hypot(target_x - local_pos.pose.pose.position.x, target_y - local_pos.pose.pose.position.y) > 0.8){
        vel_track[0] = p_xy * (target_x - local_pos.pose.pose.position.x);
        vel_track[1] = p_xy * (target_y - local_pos.pose.pose.position.y);
    }
    else{
        vel_track[0] = 2 * (target_x - local_pos.pose.pose.position.x);
        vel_track[1] = 2 * (target_y - local_pos.pose.pose.position.y);
    }

    //速度限幅，第三处修改，改为对总体速度限幅，并比例缩小,强制合速度为max
    float vel_combination=hypot(vel_track[0],vel_track[1]);
    if(vel_combination>vel_sp_max)
    {
        vel_track[0]=vel_track[0]*vel_sp_max/vel_combination;
        vel_track[1]=vel_track[1]*vel_sp_max/vel_combination;
    }
    vel_collision[0] = 0;
    vel_collision[1] = 0;
    ROS_WARN("Velocity Command Body before CA: vx: %.2f , vy: %.2f ", vel_track[0], vel_track[1]);

    // 4. 避障策略
    if (flag_collision_avoidance.data == true)
    {
        distance_cx = distance_c * cos(((float)angle_c) / 180 * 3.1415926);
        distance_cy = distance_c * sin(((float)angle_c) / 180 * 3.1415926);
        ROS_WARN("Angle_c: %d deg ", angle_c);
        ROS_WARN("angle_c/180*3.1415926: %.2f rad ", angle_c / 180 * 3.1415926);
        ROS_WARN("cos(angle_c/180*3.1415926): %.2f  ", cos(angle_c / 180 * 3.1415926));
        ROS_WARN("Distance_cx: %.2f , Distance_cy: %.2f ", distance_cx, distance_cy);

        float F_c;

        F_c = 0;

        if (distance_c > R_outside)
        {
            // 对速度不做限制
            vel_collision[0] = vel_collision[0] + 0;
            vel_collision[1] = vel_collision[1] + 0;
            cout << " Forward Outside " << endl;
        }

        // 小幅度抑制移动速度
        if (distance_c > R_inside && distance_c <= R_outside)
        {
            F_c = p_R * (R_outside - distance_c);
        }

        // 大幅度抑制移动速度
        if (distance_c <= R_inside)
        {
            F_c = p_R * (R_outside - R_inside) + p_r * (R_inside - distance_c);
        }
        ROS_WARN("Force F_c: %.2f ", F_c);

        //第一处修改，修改为更美观的写法！！！！！！！！
        vel_collision[0] = vel_collision[0] - F_c * distance_cx / distance_c;
        vel_collision[1] = vel_collision[1] - F_c * distance_cy / distance_c;
      
         //避障速度限幅，第五处修改，对避障速度限幅同第三处
        float vel_collision_combination=hypot(vel_collision[0],vel_collision[1]);
        if(vel_collision_combination>vel_collision_max)
        {
            vel_collision[0]=vel_collision[0]*vel_collision_max/vel_collision_combination;
            vel_collision[1]=vel_collision[1]*vel_collision_max/vel_collision_combination;
        }
    }
    // 5. 速度叠加，得到最终速度指令
    rotation_yaw(-yaw, vel_track, vel_track);           // 追踪速度转机体坐标系
    vel_sp_body[0] = vel_track[0] + vel_collision[0];
    vel_sp_body[1] = vel_track[1] + vel_collision[1]; // dyx

    //ROS_WARN("Velocity Command Body Track: vx: %.2f , vy: %.2f ", vel_track[0], vel_track[1]);
    //ROS_WARN("Velocity Command Body Collision: vx: %.2f , vy: %.2f ", vel_collision[0], vel_collision[1]);
    //ROS_WARN("Velocity Command Body after CA: vx: %.2f , vy: %.2f ", vel_sp_body[0], vel_sp_body[1]);

    // 找当前位置到目标点的xy差值，如果出现其中一个差值小，另一个差值大，
    // 且过了一会还是保持这个差值就开始从差值入手。
    // 比如，y方向接近0，但x还差很多，但x方向有障碍，这个时候按discx cy的大小，缓解y的难题。

    //第六处修改，总体速度限幅,同第三处
    float vel_sp_combination=hypot(vel_sp_body[0],vel_sp_body[1]);
    if(vel_sp_combination>vel_sp_max)
    {
        vel_sp_body[0]=vel_sp_body[0]*vel_sp_max/vel_sp_combination;
        vel_sp_body[1]=vel_sp_body[1]*vel_sp_max/vel_sp_combination;
    }

    rotation_yaw(yaw, vel_sp_body, vel_sp_ENU);
    setpoint_raw.type_mask = 1 + 2 /* + 4  +8 + 16 + 32 */ + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048;
    setpoint_raw.coordinate_frame = 1;
    setpoint_raw.velocity.x = vel_sp_ENU[0];
    setpoint_raw.velocity.y = vel_sp_ENU[1];
    setpoint_raw.position.z = target_z + init_position_z_take_off;
    setpoint_raw.yaw = target_yaw;

    ROS_WARN("Velocity Command ENU: vx: %.2f , vy: %.2f ", vel_sp_ENU[0], vel_sp_ENU[1]);
    ROS_WARN("Target Pos: ( %.2f, %.2f, %.2f )", target_x + init_position_x_take_off, target_y + init_position_y_take_off, target_z + init_position_z_take_off);
    ROS_WARN("Current Pos: ( %.2f, %.2f, %.2f )", local_pos.pose.pose.position.x, local_pos.pose.pose.position.y, local_pos.pose.pose.position.z);

    //ROS_INFO("fabs_x: %lf, fabs_y %lf", fabs(local_pos.pose.pose.position.x - target_x - init_position_x_take_off), fabs(local_pos.pose.pose.position.y - target_y - init_position_y_take_off));
    
    if (fabs(local_pos.pose.pose.position.x - target_x - init_position_x_take_off) < err_max && fabs(local_pos.pose.pose.position.y - target_y - init_position_y_take_off) < err_max && fabs(local_pos.pose.pose.position.z - target_z - init_position_z_take_off) < err_max && fabs(yaw - target_yaw) < 0.1)
    {
        static bool first_time = true;
        if (first_time)
        {
            ROS_INFO("到达目标点，开始避障任务完成处理");
            last_request = ros::Time::now();
            first_time = false;
        }
        ROS_INFO("到达目标点（假点/原始目标），避障任务完成");
        //
        // ========== 第七次：避障巡航到达目标点重置超时标志 ==========
        collision_cruise_flag = false;
        collision_cruise_timeout_flag = false;
        // ========== 新增结束 ==========
         if (ros::Time::now() - last_request > ros::Duration(1.0))
        {

            last_request = ros::Time::now();
            first_time = true;
            return true;
        }
        

        
    }
    return false;
}
/************************************************************************
函数 9: stuck_detection 震荡检测函数
根据位置回调数据，速度回调判断无人机是否处于震荡状态
输入参数：无人机位置，速度
返回值：true/false表示是否处于震荡状态
*************************************************************************/
int flag = 0;
bool stuck_detection(const vector<point> &pos, const vector<Vel> &vel)
{

    int n1 = pos.size();
    int n2 = vel.size();
    int n = (n1 > n2) ? n2 : n1; // 找出最小的，防止指向空值
        for (int i = 0; i < n; i++)
        {
            for (int j = i + 1; j < n; j++)
            {                                                                // 遍历任意两个点
                float dis = hypot(pos[i].x - pos[j].x, pos[i].y - pos[j].y); // 算距离
                //25.12.12(19.15) 修改震荡判断条件，增加速度反向判断
                if (dis <= 0.3 && ((vel[i].x * vel[j].x + vel[i].y * vel[j].y )<0) ) // 如果距离小于0.2米且速度反向
                    flag++;
            }
            ROS_INFO("flag = %d",flag);
        }
    
    return flag > 3; // 如果有超过6对点满足条件，则认为震荡
}

/*
函数10：计算临时避障点
 target: 目标点（世界坐标系）
 current: 当前位置（世界坐标系）
 dist: 障碍点相对机体的距离
 angle: 障碍点相对机体的角度（度）
 err: 输出错误码（CALC_SUCCESS/CALC_INVALID_PARAM）
@return: 避障点（世界坐标系）
*/

/*
输入格式：
1.target，目标点，定义代码如下
point target；
target.x=<终点的x，类型为double>
target.y=<终点的y，类型为double>
2.curent，当前位置，同理
curent.x=<>
curent.y=<>
3.dist，到最近避障点的距离
4.angle,到最近避障点的角度
5.err，错误码指针，定义代码如下
CalcErr err;

示例：



  point target = {10, 0};
    point current = {6, 0};
    CalcErr err;
    point waypoint = cal_temporary_waypoint(target, current, 5, 36, &err);



*/
float final_r = 7.0f; // 终点限制圆半径
point cal_temporary_waypoint(point target, point current, float dist, int angle, CalcErr *err)
{
    point barrier_body, barrier_world, cross_point;
    CalcErr inner_err = CALC_SUCCESS;

    // 参数合法性检查，此部分是ai加的
    if (dist < 0 || err == NULL)
    {
        *err = CALC_INVALID_PARAM;
        (point){NAN, NAN}; // 返回无效点
    }

    // 1. 计算障碍点的机体坐标
    double angle_rad =1.0*angle * M_PI / 180.0; // 角度转弧度
    barrier_body.x = dist * cos(angle_rad);
    barrier_body.y = dist * sin(angle_rad);

    // 2. 机体坐标转世界坐标（补全注释要求的逻辑）
    float yaw_rad = yaw;
    // rotation_yaw(yaw, (float[]){(float)barrier_body.x, (float)barrier_body.y}, (float[]){(float)barrier_world.x, (float)barrier_world.y});//修改注释中的问题，先进行旋转
    //手动进行旋转 25.12.12(18.58)
    float barrier_world_x = barrier_body.x * cos(yaw_rad) - barrier_body.y * sin(yaw_rad);
    float barrier_world_y = barrier_body.x * sin(yaw_rad) + barrier_body.y * cos(yaw_rad);
    barrier_world.x = barrier_body.x + current.x;
    barrier_world.y = barrier_body.y + current.y; // 既然机头不转动的话，可以直接相加求解？但是这样就和那个算法的错误一样了，暂且先这么着,已经修改了

    // 3. 计算垂直线交点
    segment seg = {current, barrier_world};
    cross_point = getCross(seg, target, &inner_err);
    *err = inner_err;

    point temp_target;
    // 4. 计算避障点
    temp_target.x = 2 * cross_point.x - target.x;
    temp_target.y = 2 * cross_point.y - target.y;


    // ========== 6. 等比缩放至目标点的限制圆内 ==========
    // 6.1 计算原始避障点到目标点的向量和距离
    float vec_tx = temp_target.x - target.x; 
    float vec_ty = temp_target.y - target.y; 
    float dist_to_target = hypot(vec_tx, vec_ty);   // 原始避障点到目标点的距离

    point temp_target_final;
    // 6.2 若距离超过限制半径，等比缩放；否则直接保留
    if (dist_to_target > final_r && dist_to_target > 1e-6)
    {                                           // 避免除零
        float scale = final_r / dist_to_target; // 缩放比例
        temp_target_final.x = target.x + vec_tx * scale;
        temp_target_final.y = target.y + vec_ty * scale;
    }
    else
    {
        temp_target_final = temp_target; // 已在圆内，无需缩放
    }

    return temp_target_final;
}

/*
函数11：计算线段seg的直线，与过point_p且垂直于该直线的交点
 seg:由当前位置和障碍点确定的线段
 point_p: 垂直线经过的点，即目标点
 err: 输出错误码（CALC_SUCCESS/CALC_DIV_ZERO）
 return: 交点（若出错返回NAN）
*/
point getCross(segment seg, point point_p, CalcErr *err)
{
    point cross = {NAN, NAN};
    float dx = seg.p1.x - seg.p2.x;
    float dy = seg.p1.y - seg.p2.y;

    // 防护：基准线段为垂直线（dx=0）
    if (fabs(dx) < 1e-8)
    {
        cross.x = seg.p1.x;
        cross.y = point_p.y;
        *err = CALC_SUCCESS;
        return cross;
    }

    // 防护：基准线段为水平线（dy=0）
    if (fabs(dy) < 1e-8)
    {
        cross.x = point_p.x;
        cross.y = seg.p1.y;
        *err = CALC_SUCCESS;
        return cross;
    }

    // 常规情况：斜率不为0且非垂直
    float slope1 = dy / dx;                            // 基准直线斜率
    float intercept1 = seg.p1.y - slope1 * seg.p1.x;   // 基准直线截距
    float slope2 = -1.0 / slope1;                      // 垂直线斜率
    float intercept2 = point_p.y - slope2 * point_p.x; // 垂直线截距

    cross.x = (intercept1 - intercept2) / (slope2 - slope1);
    cross.y = slope1 * cross.x + intercept1;
    *err = CALC_SUCCESS;

    return cross;
}
/*
使用说明

1.有关计算临时目标点的函数：
cal_temporary_waypoint(point target, point current, double dist, double angle, CalcErr *err)
point target: 目标点（世界坐标系）
需要看一下结构体的定义，传入一对x,y坐标
point current: 当前位置（世界坐标系）
double dist: 障碍点相对机体的距离
double angle: 障碍点相对机体的角度（度）
CalcErr *err: 输出错误码（CALC_SUCCESS/CALC_INVALID_PARAM）
返回值: 避障点（世界坐标系）



2.超时阈值配置说明：
mission_cruise_timeout: 12.0   # 普通巡航超时阈值（秒）
collision_cruise_timeout: 18.0 # 避障巡航超时阈值（秒）

*/
