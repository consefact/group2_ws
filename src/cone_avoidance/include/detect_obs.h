#include<vector>
#include<queue>
#include<random>
#include<Eigen/Dense>
// #include<ros/ros.h>
// #include<mavros_msgs/PositionTarget.h>
#include<livox_ros_driver/CustomMsg.h>
#include<nav_msgs/Odometry.h>
/************************************************************************
lidar_cb
点云回调函数，处理Livox雷达的点云数据，实现360度障碍物检测
/livox/lidar example:

*************************************************************************/
extern nav_msgs::Odometry local_pos;
double yaw=local_pos.pose.pose;
extern float if_debug;
extern float init_position_z_take_off;

std::random_device rd;
std::mt19937 gen(rd());

float eps=0.05;          //评判噪点和簇边界和扩簇距离的范围半径
int min_pts=5;        //评判噪点和簇边界点的最少点数
float max_radius=30;   //评判圆是否是直线墙面的最大半径
float drone_safe_radius=0.3;//无人机安全半径

float px=local_pos.pose.pose.position.x;
float py=local_pos.pose.pose.position.y;

struct Cluster{
    int id;
    std::vector<float> center;
    float r;
    float safe_radius;
    Cluster(int id):id(id),center{0,0},r(0),safe_radius(0){}
};
std::vector<Cluster> obstacles;

//转坐标系函数（机体->全局），如果重复可以更换
static std::vector<float> my_rotation_yaw_position(double yaw_angle,float x,float y)
{
    std::vector<float> output(2);
    output[0] = x * cos(yaw_angle) - y * sin(yaw_angle);
    output[1] = x * sin(yaw_angle) + y * cos(yaw_angle);
    return output;
}

//定义墙体
struct StraightWall{
    std::vector<float>close_point,left_point,right_point;
    StraightWall(float cx=-30.0,float cy=0,float lx=0,float ly=0,float rx=0,float ry=0)
        :close_point{cx,cy},left_point{lx,ly},right_point{rx,ry}{}
    float distance_to_drone(){
        float dx=close_point[0]-px;
        float dy=close_point[1]-py;
        return sqrt(dx*dx+dy*dy);
    }
    friend class DBSCAN;
};
std::vector<StraightWall> straight_walls;

struct SurroundWall{
    std::vector<float>close_point;

    SurroundWall(float cx=-30.0,float cy=0)
        :close_point{cx,cy}{}
    float distance_to_drone(){
        float dx=close_point[0]-px;
        float dy=close_point[1]-py;
        return sqrt(dx*dx+dy*dy);
    }
    friend class DBSCAN;
};
SurroundWall surround_wall;
//匿名命名空间
namespace detect_obs{

    //临时三维点
    struct PointTemp3{
        float x,y,z;
        PointTemp3(float x,float y,float z):x(x),y(y),z(z){}
    };
    std::vector<PointTemp3> points_temp3;
    
    //无人机（x,y）坐标，如重复可删

    // float pz=local_pos.pose.pose.position.z;
    // float ow=local_pos.pose.pose.orientation.w;
    // float ox=local_pos.pose.pose.orientation.x;
    // float oy=local_pos.pose.pose.orientation.y;
    // float oz=local_pos.pose.pose.orientation.z;
    

    //为实现欧几里德聚类使用的点集
    class Point{
    public:
        float x,y;
        bool noise;
        bool few_close;
        bool visited;

        Point(float x=0,float y=0):
            x(x),y(y),noise(false),
            visited(false),few_close(false){}

        float dist_to(const Point&other)const{
            float dx=x-other.x;
            float dy=y-other.y;
            return sqrt(dx*dx+dy*dy);
        }
        float distance(){
            return sqrt(x*x+y*y);
        }
        friend class DBSCAN;
        bool operator==(const Point&other)const{
            return x==other.x&&y==other.y;
        }
        using RefPoints=std::vector<std::reference_wrapper<Point>>;
    };

    //RANSAC算法拟合直线，但是太麻烦放弃了

    //使用Point的转坐标系重载
    static Point my_rotation_yaw(double yaw_angle,Point&input)
    {
        Point output;
        float x{input.x};
        float y{input.y};
        output.x = x * cos(yaw_angle) - y * sin(yaw_angle);
        output.y = x * sin(yaw_angle) + y * cos(yaw_angle);
        return output;
    }

    //更新wall的最近点
    void update(SurroundWall wall,Point& other){
        if(wall.distance_to_drone()<other.distance()){
            wall.close_point=my_rotation_yaw_position(yaw,other.x,other.y);
        }
    }

    //为计算圆制定对应结构体
    struct Circle{
        float radius;
        Point center;
    };

    //计算三点圆(算法AI给)
    Circle three_points_round(Point& point1,Point& point2,Point& point3){
        Circle circle;
        double x1(point1.x),x2(point2.x),x3(point3.x),y1(point1.y),y2(point2.y),y3(point3.y);
        double A=x1*(y2-y3)-y1*(x2-x3)+x2*y3-x3*y2;
        float B=(x1*x1+y1*y1)*(y3-y2)+(x2*x2+y2*y2)*(y1-y3)+(x3*x3+y3*y3)*(y2-y1);
        float C=(x1*x1+y1*y1)*(x2-x3)+(x2*x2+y2*y2)*(x3-x1)+(x3*x3+y3*y3)*(x1-x2);
        // 圆心坐标
        double center_x = -B / (2 * A);
        double center_y = -C / (2 * A);
        circle.center.x=static_cast<float>(center_x);
        circle.center.y=static_cast<float>(center_y);
        // 半径
        double dx = x1 - center_x;
        double dy = y1 - center_y;
        circle.radius=static_cast<float>( std::sqrt(dx*dx + dy*dy) );
        return circle;
    }

    //计算两点圆,后续可以考虑再造一个切线圆
    Circle two_points_round(Point& point1,Point& point2)
    {
        float x1(point1.x),x2(point2.x),y1(point1.y),y2(point2.y);
        Circle circle;
        circle.center.y=((x1+x2)/2);
        circle.center.x=((y1+y2)/2);
        float dx(x1-x2);
        float dy(y1-y2);
        circle.radius=sqrt(dx*dx+dy*dy);
        return circle;
    }

    //计算圆心，半径，顺便判定是否是墙体
    bool cul_r_center(Cluster& cluster,Point::RefPoints cluster_points){
        int count_points{static_cast<int>(cluster_points.size())};
        Point closest_point;
        Point leftest_point;
        Point rightest_point;
        float min_distance{100};
        //正为逆时针，负为顺时针
        float leftest_angle{0};
        float rightest_angle{0};
        float x_points_cen{0};
        float y_points_cen{0};
        for(auto point:cluster_points){
            x_points_cen+=point.get().x;
            y_points_cen+=point.get().y;
        }
        x_points_cen/=count_points;
        y_points_cen/=count_points;
        for(auto point:cluster_points){
            float x=point.get().x,y=point.get().y;
            float distance=point.get().distance();
            float angle=atan2(x_points_cen*y-y_points_cen*x,x_points_cen*x+y_points_cen+y);
            if(distance<min_distance){
                min_distance=distance;
                closest_point=point.get();
            }
            if(angle>leftest_angle){
                leftest_angle=angle;
                leftest_point=point.get();
            }
            if(angle>rightest_angle){
                rightest_angle=angle;
                rightest_point=point.get();
            }
        }
        float delta_angle(leftest_angle-rightest_angle);
        Circle circle2=two_points_round(leftest_point,rightest_point);
        Circle circle3=three_points_round(closest_point,leftest_point,rightest_point);

        //直线墙体条件：圆大小大到一定程度（直线），
        if(circle3.radius>max_radius){
            straight_walls.emplace_back(closest_point.x,closest_point.y,leftest_point.x,leftest_point.y,rightest_point.x,rightest_point.y);
            return false;
        //周围墙条件：自身在圆内（可能是被墙包围或者内凹型障碍物）并且左右角度大于180（排除内凹型障碍物）
        }else if(circle3.center.distance()<circle3.radius&&delta_angle>M_PI){
            update(surround_wall,closest_point);
            return false;
        }else{
            //取近圆作为障碍物的圆
            Circle&circle=(circle2.center.distance()<circle3.center.distance())?circle2:circle3;
            circle.center=my_rotation_yaw(yaw,circle.center);
            cluster.center={circle.center.x+px,circle.center.y+py};
            cluster.r=circle.radius;
            cluster.safe_radius=cluster.r+drone_safe_radius;
            return true;
        }
    }
}



//实现DBSCAN的类
class DBSCAN{
private:
    //每簇点集
    std::vector<detect_obs::Point> points;
private:
    //加入点
    void add_point(float x,float y){
        points.emplace_back(x,y);
    }
    void add_point(const detect_obs::Point& p){
        points.push_back(p);
    }
    //取附近的点
    std::vector<int> get_close_points(int index){
        std::vector<int>close_points;
        const detect_obs::Point& p=points[index];
        for(int i(0);i<points.size();i++){
            if(i==index) continue;
            if(p.dist_to(points[i])<=eps){
                close_points.push_back(i);
            }
        }
        return close_points;
    }
    detect_obs::Point::RefPoints get_close_points(const detect_obs::Point& targer_p){
        detect_obs::Point::RefPoints close_points;
        for(auto&p:points){
            if(p==targer_p)continue;
            if(p.dist_to(targer_p)<=eps){
                close_points.push_back(p);
            }
        }
        return close_points;
    }
    //取附近未加入簇的点
    auto restricted_close(const detect_obs::Point& target_p){
        detect_obs::Point::RefPoints close_points;
        for(auto&p:points){
            if(!p.visited&&p.dist_to(target_p)<=eps){
                close_points.push_back(p);
            }
        }
        return close_points;
    }
    //计算周围点是不是少于min_pts;
    bool few_close_points(std::vector<int> close_points){return close_points.size()<min_pts;}
    bool few_close_points(int index){//可能更快的函数
        int close_points_count{-1};//排除自己
        const detect_obs::Point& index_point=points[index];
        for(const auto& p:points){
            if(p.dist_to(index_point)<=eps){
                close_points_count++;
            }
        }
        return close_points_count<min_pts;
    }
    bool few_close_points(const detect_obs::Point& target_point){
        int close_points_count{-1};
        for(const auto& p:points){
            if(p.dist_to(target_point)<=eps){
                close_points_count++;
            }
        }
        return close_points_count<min_pts;
    }


    //取这一点的簇，识别为墙体则返回false，障碍物则为true
    bool make_cluster(detect_obs::Point& first_point,int cluster_id){
        using QueRefPoints=std::queue<std::reference_wrapper<detect_obs::Point>>;
        Cluster cluster(cluster_id);//创建簇
        detect_obs::Point::RefPoints cluster_points;//簇的点集
        QueRefPoints seeds;//种子队列，种子作为中心不断扩大簇并播种
        cluster_points.push_back(first_point);
        seeds.push(first_point);
        while(!seeds.empty()){
            auto p=seeds.front();
            seeds.pop();
            auto close_points=restricted_close(p);
            for(auto&q:close_points){
                q.get().visited=1;
                if(!q.get().few_close){
                    seeds.push(q);
                }
                cluster_points.push_back(q);
            }
        }
        if(cul_r_center(cluster,cluster_points)){
            obstacles.emplace_back(cluster);
            return true;
        }
        return false;
    }

public:
    void get_clusters(){
        for(const auto&pt:detect_obs::points_temp3){
            add_point(pt.x,pt.y);
        }
        int cluster_id=1;
        for(auto &p:points)
        {
            if(p.visited){
                continue;
            }
            auto close_points=get_close_points(p);
            if(few_close_points(p)){
                p.few_close=1;
                continue;
            }
            if(make_cluster(p,cluster_id)){
                cluster_id++;
            }
        }
    }
    // friend void livox_custom_cb(const livox_ros_driver::CustomMsg::ConstPtr& livox_msg);
};

DBSCAN dbscan;

constexpr float height_threshold_value{0.05};
int plane_points{0};

void livox_custom_cb(const livox_ros_driver::CustomMsg::ConstPtr& livox_msg) {
    // 初始化bins
    // ROS_INFO("Received Livox point cloud with %d points", livox_msg->point_num);
    detect_obs::points_temp3.clear();
    obstacles.clear();
    straight_walls.clear();
    surround_wall.close_point = {-30.0f, 0.0f};   
    plane_points=0;

    int total_points = livox_msg->point_num;

    // 遍历Livox自定义消息中的点
    for (const livox_ros_driver::CustomPoint& point :livox_msg->points) {
        
        // 筛选0度平面附近的点
        if (fabs(point.z) > height_threshold_value) {
            continue;
        }

        float x = point.x;
        float y = point.y;
        float z = point.z;
        
        plane_points++;

        std::uniform_int_distribution<> dis_int(1,5);

        if(dis_int(gen)==1){
            detect_obs::points_temp3.emplace_back(x,y,z);
        }
    }
    
    dbscan.get_clusters();

    // if(if_debug==1){
    //     // 打印点云统计信息
    //     ROS_INFO("==========================================");
    //     ROS_INFO("点云处理统计:");
    //     ROS_INFO("原始点数: %d, 平面筛选后点数: %d", total_points, plane_points);
    //     ROS_INFO("DBSCAN检测结果:");
    //     ROS_INFO("簇数量: %zu", obstacles.size());
    //     ROS_INFO("直线墙数量: %zu", straight_walls.size());
        
    //     // 打印每个障碍物簇的详细信息
    //     for(size_t i = 0; i < obstacles.size(); ++i) {
    //         const Cluster& cluster = obstacles[i];
    //         ROS_INFO("簇[%d]: 圆心(全局坐标) (%.2f, %.2f), 半径: %.3fm, 安全半径: %.3fm", 
    //                 cluster.id, 
    //                 cluster.center[0], 
    //                 cluster.center[1],
    //                 cluster.r,
    //                 cluster.safe_radius);
    //     }
        
    //     // 打印直线墙信息
    //     for(size_t i = 0; i < straight_walls.size(); ++i) {
    //         StraightWall& wall = straight_walls[i];
    //         float distance = wall.distance_to_drone();
    //         ROS_INFO("直线墙[%zu]: 最近点(%.2f, %.2f), 距离无人机: %.2fm", 
    //                 i, wall.close_point[0], wall.close_point[1], distance);
    //     }
        
    //     // 打印周围墙信息
    //     if(std::isfinite(surround_wall.close_point[0]) && std::isfinite(surround_wall.close_point[1])) {
    //         float distance = surround_wall.distance_to_drone();
    //         ROS_INFO("周围墙: 最近点(%.2f, %.2f), 距离无人机: %.2fm", 
    //                 surround_wall.close_point[0], surround_wall.close_point[1], distance);
    //     }
        
    //     // 无人机当前位置
    //     ROS_INFO("无人机位置: (%.2f, %.2f), 偏航角: %.2f度", 
    //             px, py, yaw * 180.0 / M_PI);
    //     ROS_INFO("==========================================");
    // }
}