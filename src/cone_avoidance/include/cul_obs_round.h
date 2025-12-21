#include<Eigen/Dense>
#include<vector>
#include"new_detect_obs.h"
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

void transObs(const std::vector<Obstacle>& obss){
    obs_rounds.clear();
    for(auto &obs:obss){
        ObsRound obs_round;
        obs_round.position=obs.position;
        obs_round.radius=obs.radius;
        obs_round.safe_radius=obs.radius+UAV_radius;
        Eigen::Vector2f dist_vec=obs.position-current_pos;
        float dist=dist_vec.norm();
        Eigen::Vector2f radius_vec=dist_vec/dist*obs_round.safe_radius;
        if(dist<=obs_round.safe_radius){
            Eigen::Rotation2D<float> rot(M_PI/2);
            Eigen::Vector2f close_point=obs.position-radius_vec;
            obs_round.left_point=rot*radius_vec/2+close_point;
            obs_round.right_point=rot.inverse()*radius_vec/2+close_point;
        }else{
            float angle=asin(obs_round.safe_radius/dist);
            Eigen::Rotation2D<float> rot(M_PI/2-angle);
            obs_round.right_point=rot*(-radius_vec)+obs.position;
            rot=Eigen::Rotation2D<float>(-M_PI/2+angle);
            obs_round.left_point=rot*(-radius_vec)+obs.position;
        }
        obs_rounds.push_back(obs_round);
    }
}