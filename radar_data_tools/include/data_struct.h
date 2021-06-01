#include <iostream>
#include <vector>

struct GPSinsData
{
    double time_stamp;
    double time_s;
    double lat;     //纬度
    double lon;     //经度
    double height;  //高度
    double x;
    double y;
    double z;
    double vx;
    double vy;
};

struct RadarData
{
    double time;
    double x_radar; 
    double y_radar;
    double x_in_world;     //局部东北天坐标系下的x
    double y_in_world;     //局部东北天坐标系下的y
    double r;       // 测量距离
    double angle;   // 目标方位角，正前方为0，右侧为正。
    double vx;      //径向速度
    double vy;
    double rcs;
    int track_id;
    int obj_id;
};

struct RadarDataTime
{
    long double time;
    std::vector<RadarData> radar_datas;
    unsigned int TimeS;    //时间，单位：秒
    unsigned int TimeNS;   //时间，单位：纳秒
};

struct MatchedCalibData
{
    double r;
    double angle;
    double lat;
    double lon;
    double height;
};