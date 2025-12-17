//#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>
#include <iostream>


struct Obstacle
{
    Eigen::Vector2f position;
    float radius;
};

std::vector<Obstacle> obstacles;
/*
 * ===================== 参数定义 =====================
 */

// CBF 参数
const float ALPHA = 2.0;
const float MAX_SPEED = 2.0;
const float OBS_EPS = 0.1;
const float KV = 0.2;
const float KN = 0.1;
const float W_goal = 0.6;
const float W_free = 0.4;


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
    const Eigen::Vector2f& target,
    const Eigen::Vector2f& UAV_pos,
    const Eigen::Vector2f& UAV_vel,
    const std::vector<Obstacle>& obstacles,
    float UAV_radius)
{
    /* ================= 1. 名义控制：指向目标 ================= */
    Eigen::Vector2f u_dir = target - UAV_pos;      // 指向目标的位移向量
    if (u_dir.norm() > 1e-3)
        u_dir.normalize();                         // 单位方向
    Eigen::Vector2f u = MAX_SPEED * u_dir;         // 名义速度 u_des


    /* ================= 2. 激活障碍物筛选 ================= */
    std::vector<Obstacle> active_obs;

    for (const auto& obs : obstacles)
    {
        Eigen::Vector2f r_a = obs.position - UAV_pos;   // UAV -> 障碍物
        float R_sq = (UAV_radius + obs.radius) *
                     (UAV_radius + obs.radius);         // 安全半径平方

        // CBF 函数 h(x) = ||r||^2 - (2R)^2
        // 提前放大安全区域，用于更早介入避障
        float h = r_a.squaredNorm() - 4.0f * R_sq;

        // h_dot = ∇h · x_dot = -2 r^T v
        float h_dot = -2.0f * r_a.dot(UAV_vel);

        // 距离过近 且 正在靠近 → 激活该障碍物
        if (h < 0 && h_dot < OBS_EPS)
            active_obs.push_back(obs);
    }


    /* ================= 3. 自由空间方向（避障启发） ================= */
    Eigen::Vector2f free_dir = Eigen::Vector2f::Zero();
    for (const auto& obs : active_obs)
    {
        Eigen::Vector2f r = obs.position - UAV_pos;
        // 人工势场式排斥方向，指向远离障碍的自由空间
        free_dir -= r / (r.squaredNorm() + 1e-3f);
    }
    if (free_dir.norm() > 1e-3)
        free_dir.normalize();


    /* ================= 4. 构造参考速度 ================= */
    // 参考控制不是最终解，而是 CBF 约束投影的“目标点”
    Eigen::Vector2f u_ref =
        W_goal * u +                      // 目标跟踪项
        W_free * free_dir * u.norm();     // 避障引导项


    /* ================= 5. 修正权重（数值稳定） ================= */
    // 障碍物越多、速度越大 → 修正越平缓
    float w = 1.0f
            + KV * UAV_vel.norm()
            + KN * active_obs.size();


    /* ================= 6. CBF 安全约束投影 ================= */
    for (const auto& obs : active_obs)
    {
        Eigen::Vector2f r = obs.position - UAV_pos;
        float dist_sq = r.dot(r);
        float R_sq = (UAV_radius + obs.radius) *
                     (UAV_radius + obs.radius);

        // h(x) = ||r||^2 - R^2
        // -∇h = 2r
        Eigen::Vector2f grad_ds = 2.0f * r;

        // α h(x)，CBF 约束的松弛项
        float cbf = ALPHA * (dist_sq - R_sq);

        // 违反量：cbf + ∇h^T u_ref
        float violation = cbf - grad_ds.dot(u_ref);

        // 若违反 ∇h^T u ≥ -αh，则进行投影修正
        if (violation > 0.0f)
        {
            // 单线性约束 QP 的解析投影解
            float correction = violation / grad_ds.dot(grad_ds);
            u_ref = u_ref - correction * grad_ds / w;
        }
    }


    /* ================= 7. 速度限幅 ================= */
    if (u_ref.norm() > MAX_SPEED)
        return u_ref.normalized() * MAX_SPEED;

    return u_ref;
}
int main()
{
    Eigen::Vector2f uav_pos(2.0, 0.0);
    Eigen::Vector2f uav_vel(0.3, 0.0);
    Eigen::Vector2f target(5.0, 0.0);

    Obstacle obs;
    obs.position = Eigen::Vector2f(0.5, 0.5);
    obs.radius = 0.5;

    std::vector<Obstacle> obstacles;
    obstacles.push_back(obs);

    float uav_radius = 0.3;

    Eigen::Vector2f u_safe = applyCBF(
        target,
        uav_pos,
        uav_vel,
        obstacles,
        uav_radius
    );

    std::cout << "Safe velocity: "
              << u_safe.transpose()
              << " | norm = "
              << u_safe.norm()
              << std::endl;
    return 0;
}


