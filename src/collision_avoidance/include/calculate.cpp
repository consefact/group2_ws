#include <math.h>
#include <stdio.h>
#include <assert.h>

// 定义_GNU_SOURCE以启用M_PI
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
/*复制说明，从第九行开始到119行结束*/

/* 点结构体 */
typedef struct point
{
    double x;
    double y;
} point;

/* 线段结构体（segment） */
typedef struct segment
{
    point p1;
    point p2; // 两点确定一条直线
} segment;

// 错误码定义
typedef enum
{
    CALC_SUCCESS = 0,
    CALC_DIV_ZERO = 1,     // 除以零错误
    CALC_INVALID_PARAM = 2 // 参数非法
} CalcErr;
point getCross(segment seg, point point_p, CalcErr *err);

/*
函数9：计算临时避障点
@param target: 目标点（世界坐标系）
@param current: 当前位置（世界坐标系）
@param dist: 障碍点相对机体的距离
@param angle: 障碍点相对机体的角度（度）
@param err: 输出错误码（CALC_SUCCESS/CALC_INVALID_PARAM）
@return: 避障点（世界坐标系）
*/
point cal_temporary_waypoint(point target, point current, double dist, double angle, CalcErr *err)
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
    double angle_rad = angle * M_PI / 180.0; // 角度转弧度
    barrier_body.x = dist * cos(angle_rad);
    barrier_body.y = dist * sin(angle_rad);

    // 2. 机体坐标转世界坐标（补全注释要求的逻辑）

    barrier_world.x = barrier_body.x + current.x;
    barrier_world.y = barrier_body.y + current.y; // 既然机头不转动的话，可以直接相加求解？但是这样就和那个算法的错误一样了，暂且先这么着

    // 3. 计算垂直线交点
    segment seg = {current, barrier_world};
    cross_point = getCross(seg, target, &inner_err);
    *err = inner_err;

    return cross_point;
}

/*
函数10：计算线段seg的直线，与过point_p且垂直于该直线的交点
 seg:由当前位置和障碍点确定的线段
 point_p: 垂直线经过的点，即目标点
 err: 输出错误码（CALC_SUCCESS/CALC_DIV_ZERO）
 return: 交点（若出错返回NAN）
*/
point getCross(segment seg, point point_p, CalcErr *err)
{
    point cross = {NAN, NAN};
    double dx = seg.p1.x - seg.p2.x;
    double dy = seg.p1.y - seg.p2.y;

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
    double slope1 = dy / dx;                            // 基准直线斜率
    double intercept1 = seg.p1.y - slope1 * seg.p1.x;   // 基准直线截距
    double slope2 = -1.0 / slope1;                      // 垂直线斜率
    double intercept2 = point_p.y - slope2 * point_p.x; // 垂直线截距

    cross.x = (intercept1 - intercept2) / (slope2 - slope1);
    cross.y = slope1 * cross.x + intercept1;
    *err = CALC_SUCCESS;

    return cross;
}

/*使用说明
需要传入point target和current，还有定义err*/
int main()
{
    point target = {10, 0};
    point current = {6, 0};
    CalcErr err;
    point waypoint = cal_temporary_waypoint(target, current, 5, 36, &err);
    if (err == CALC_SUCCESS)
    {
        printf("避障点：(%.2f, %.2f)\n", waypoint.x, waypoint.y);
    }
    else
    {
        printf("计算失败，错误码：%d\n", err);
    }
    return 0;
}