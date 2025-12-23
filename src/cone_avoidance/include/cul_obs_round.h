#include<Eigen/Dense>
#include<vector>
#include"new_detect_obs.h"
#include <algorithm>

extern float UAV_radius;
extern Eigen::Vector2f current_pos;
extern std::vector<Obstacle>obstacles;
struct ObsRound
{
    Eigen::Vector2f position;
    float radius;
    float safe_radius;
    Eigen::Vector2f left_point;
    Eigen::Vector2f right_point;
};
std::vector<ObsRound> obs_rounds;

void transObs(const std::vector<Obstacle>& obss) {
    obs_rounds.clear();
    
    for (const auto &obs : obss) {
        ObsRound obs_round;
        obs_round.position = obs.position;
        obs_round.radius = obs.radius;
        // 增加 0.15m 的额外膨胀，作为缓冲区
        obs_round.safe_radius = obs.radius + 1.5*UAV_radius; 

        Eigen::Vector2f dist_vec = obs.position - current_pos;
        float dist = dist_vec.norm();
        
        // 1. 极小距离保护：防止除以零
        if (dist < 1e-3f) {
            dist_vec = Eigen::Vector2f(1.0f, 0.0f); // 给一个默认方向
            dist = 1.0f;
        }

        // 2. 侵入检测与处理 (Inside the safe circle)
        if (dist <= obs_round.safe_radius) {
            // 紧急情况：无人机已在安全区内。
            // 策略：计算垂直于连线的方向作为切点，迫使无人机切向飞出，而不是后退。
            Eigen::Vector2f dir_norm = dist_vec.normalized();
            Eigen::Vector2f tangent_dir(-dir_norm.y(), dir_norm.x()); // 旋转90度

            // 左切点：当前位置 + 左切向 * 逃逸距离
            obs_round.left_point = current_pos + tangent_dir * obs_round.safe_radius;
            // 右切点：当前位置 - 左切向 * 逃逸距离
            obs_round.right_point = current_pos - tangent_dir * obs_round.safe_radius;
        } 
        else {
            // 3. 常规切点计算 (Outside the safe circle)
            // 核心修复：使用 std::clamp 防止浮点误差导致的 asin(>1) = NaN
            float sin_theta = std::clamp(obs_round.safe_radius / dist, -1.0f, 1.0f);
            float theta = std::asin(sin_theta);

            // 旋转矩阵计算
            float c = std::cos(theta);
            float s = std::sin(theta); // 即 sin_theta

            // 向量旋转公式: x' = x*c - y*s, y' = x*s + y*c
            // 这里我们旋转的是 "反向距离向量" (-dist_vec)
            Eigen::Vector2f anti_dist = -dist_vec;

            // 右切点 (旋转 -theta)
            // 注意：因为我们要找切点，其实是旋转 (90 - theta) 或者利用几何关系
            // 这里采用标准的旋转向量法求切点位置：
            // 切点 P = Center + R * Rotation(±(90-alpha))
            // 更简单的矢量法：
            Eigen::Rotation2D<float> rot_l(theta);
            Eigen::Rotation2D<float> rot_r(-theta);
            
            // 计算切线切点相对于圆心的向量
            // 几何关系：切点连线与由圆心指向无人机的向量夹角为 (90 + theta) ? 
            // 修正逻辑：asin(r/d) 求出的是半角。
            // 正确的切点计算逻辑：
            float tangent_len = std::sqrt(dist*dist - obs_round.safe_radius * obs_round.safe_radius);
            
            Eigen::Rotation2D<float> rot_tangent_l(-theta); // 视线方向旋转 theta
            Eigen::Vector2f dir_l = rot_tangent_l * dist_vec.normalized();
            obs_round.left_point = current_pos + dir_l * tangent_len;

            Eigen::Rotation2D<float> rot_tangent_r(theta);
            Eigen::Vector2f dir_r = rot_tangent_r * dist_vec.normalized();
            obs_round.right_point = current_pos + dir_r * tangent_len;
            
            // 4. 切点外推 (Look-ahead)
            // 让目标点稍微延伸一点，避免无人机刚到切点就急停
            obs_round.left_point += (obs_round.left_point - current_pos).normalized() * 0.5f;
            obs_round.right_point += (obs_round.right_point - current_pos).normalized() * 0.5f;
            // 在 transObs 函数中修改
            // 原本：best_pt = obs.position + rot * direction
            // 修改：外推缓冲区
            float extra_buffer = 0.4f; // 物理冗余量
            obs_round.left_point += (obs_round.left_point - obs_round.position).normalized() * extra_buffer;
            obs_round.right_point += (obs_round.right_point - obs_round.position).normalized() * extra_buffer;
        }

        obs_rounds.push_back(obs_round);
    }
}