
// 传入当前位置current，safe circle的圆心，半径
#include <iostream>
#include <vector>
#define _USE_MATH_DEFINES // 必须放在cmath之前
#include <cmath>

/*
计算当前点与安全圆的切线和连心线的夹角（角度制），适配 “外点 - 圆心 - 切点” 直角三角形场景：
点在圆内：无有效切线，返回 NAN；
点在圆上：切线与半径垂直，返回 90°；
点在圆外：返回切线与 “当前点 - 圆心” 连心线的夹角（范围 0°~90°）。

传入
current	std::vector<float>	当前点的二维坐标 (x, y)	必须包含且仅包含 2 个元素
circle_center	std::vector<float>	安全圆的圆心二维坐标 (x, y)	必须包含且仅包含 2 个元素
radius	float	安全圆的半径	非负数（负数无几何意义）

返回值
taper_angle	float	切线与连心线的夹角（角度制）	点在圆内时返回 NAN，点在圆上时返回 90°，点在圆外时返回 0°~90°之间的值
*/
float give_angle_vector_calculation(std::vector<float> current,
                                    std::vector<float> circle_center,
                                    float radius)
{
    float taper_angle = NAN;
    float distance_between_centers = sqrt(pow(current[0] - circle_center[0], 2) + pow(current[1] - circle_center[1], 2)); // 计算当前点与圆心的距离
    if (distance_between_centers < radius - 1e-6)
    {
        // point is inside the circle , return NAN
        std::cout << "点在圆内，无有效结果" << std::endl;
        return taper_angle;
    }
    // 点在圆上时，夹角为90°（切线⊥半径）
    else if (fabs(distance_between_centers - radius) < 1e-6)
    {
        taper_angle = 90.0f;
        return taper_angle;
    }
    else
    {
        // calculate
        taper_angle = asin(radius / distance_between_centers); // 计算切线与连心线的夹角
        taper_angle = taper_angle * 180.0 / static_cast<float>(M_PI); // 转换为角度制
        return taper_angle;
    }
}
/*
函数功能
基于向量点积公式计算两个二维向量的夹角（角度制），适配任意非零二维向量场景：
自动校验零向量、维度异常等无效场景，返回 NAN；
对浮点误差导致的余弦值超出[-1,1]范围做钳位，避免计算异常；
返回夹角范围为 0°~180°（点积特性，无方向区分）。

传入
current_to_circle	std::vector<float>	第一个二维向量 (x, y)	必须包含且仅包含 2 个元素
current_to_speed	std::vector<float>	第二个二维向量 (x, y)	必须包含且仅包含 2 个元素

传出
夹角（0°~180°）
*/

//点积计算角度
float give_angle_vector_dot(std::vector<float> current_to_circle,
                            std::vector<float> current_to_speed)
                            {
                                float dot_product = current_to_circle[0] * current_to_speed[0] + current_to_circle[1] * current_to_speed[1]; // 计算两向量的点积

                                float magnitude_circle = hypotf(current_to_circle[0], current_to_circle[1]); // 计算两向量的模长（欧几里得长度）
                                float magnitude_speed = hypotf(current_to_speed[0], current_to_speed[1]);


                                const float EPS = 1e-6f; // 浮点精度容错阈值（避免极小值导致的计算异常）
                                if (magnitude_circle < EPS || magnitude_speed < EPS)
                                {
                                    std::cerr << "[错误] 存在零向量（模长为0），无法计算夹角！" << std::endl;
                                    return NAN;
                                }

                                // ===================== 6计算cosθ，并钳位到[-1,1] =====================
                                // 浮点误差可能导致cosθ超出[-1,1]，acos会返回NAN，因此必须钳位
                                float cos_theta = dot_product / (magnitude_circle * magnitude_speed);
                                cos_theta = std::max(std::min(cos_theta, 1.0f), -1.0f); // 限制范围在[-1,1]


                                float angle = acosf(cos_theta);                     // acosf：float版本的反余弦函数
                                angle = angle * 180.0 / static_cast<float>(M_PI);
                                return angle;
                            }

    // 测试示例
    int main()
{
    std::vector<float> current = {0.0, 0.0};
    std::vector<float> circle_point = {0.0, 10.0f};
    float radius = 5.0f;
    float angle = give_angle_vector_calculation(current, circle_point, radius);
    printf("%lf\n", angle);
    std::vector<float> circle = {2.0f, 2.0f};
    std::vector<float> speed = {1.0f, 0.0f};
    float angle_dot = give_angle_vector_dot(circle, speed);
    printf("%lf", angle_dot);

    return 0;
}