#ifndef DATA_IO_HPP
#define DATA_IO_HPP

#include <string>
#include <vector>
#include <memory>
#include <opencv2/opencv.hpp>

enum ObjType
{
  OBJ_TYPE_CAR = 0,
  OBJ_TYPE_PEOPLE =1,

  OBJ_TYPE_MAX,
};


struct SensorPairs
{
  std::string cam_name = "None";
  std::string radar_name = "None";
  std::string lidar_name = "None"; 
};

struct RadarData
{
  double x_radar; 
  double y_radar;
  double r;       // 测量距离
  double angle;   // 目标方位角，正前方为0，右侧为正。
  double vx;      //径向速度
  double vy;
  double rcs;
  int track_id;
  int obj_id;
};

struct LidarData
{
  float x;
  float y;
  float z;
  float intensity;
};

struct Point2F
{
    float x;
    float y;
};

struct DataPairs
{
  SensorPairs sensor_pairs;

  cv::Mat image; // TODO
  std::vector<RadarData> radar_data; // TODO
  std::vector<LidarData> lidar_data; // 原始点云数据

  unsigned int imTimeS;    //时间，单位：秒 image
  unsigned int imTimeNS;   //时间，单位：纳秒 image

  unsigned int radarTimeS;    //时间，单位：秒 
  unsigned int radarTimeNS;   //时间，单位：纳秒 

  unsigned int lidarTimeS;    //时间，单位：秒 
  unsigned int lidarTimeNS;   //时间，单位：纳秒 

};

struct BoundingBoxTrans // 不需要转换 
{
  std::string sensor_name;
  std::string license_plate = "None";  
  float x; // x-component of top left coordinate
  float y; // y-component of top left coordinate
  float w; // width of the box
  float h; // height of the box
  float score; // score of the box;

  float x_predict; // predict x-component of top left coordinate
  float y_predict; // predict y-component of top left coordinate
  float w_predict; // predict width of the box
  float h_predict; // predict height of the box

  unsigned int track_id;
  unsigned int obj_id;
  int untracked_num;

  float utm_x;
  float utm_y;

  double x_w;
  double y_w;
  double yaw;
  double vx;
  double vy;
  double wz;
  
  double length;
  double width;
  double height;


  double x_w_predict;
  double y_w_predict;

  int cerred = 0;
  unsigned int TimeS;    //时间，单位：秒
  unsigned int TimeNS;   //时间，单位：纳秒

};

struct RadarTrackings
{
  std::string sensor_name; // TODO remove

  std::string fusion_sensor;
  Point2F uv_project; // 图像上的点
  
  float confidence;

  unsigned int track_id;
  unsigned int obj_id;


  // TODO float
  double r;
  double angle;
  double vx;      //径向距离
  double vy;
  double x_radar;
  double y_radar;

  double x_w;
  double y_w;

  double x_w_predict;
  double y_w_predict;

  unsigned int TimeS;    //时间，单位：秒 TODO remove
  unsigned int TimeNS;   //时间，单位：纳秒 TODO remove
};


struct MidResultsPairs
{
  SensorPairs sensor_pairs;

  std::vector<BoundingBoxTrans> boxes_cam;
  std::vector<RadarTrackings> targets_radar;
  std::vector<LidarData> lidar_data; // 包含目标的点云

  unsigned int imTimeS;    //时间，单位：秒 image
  unsigned int imTimeNS;   //时间，单位：纳秒 image

  unsigned int radarTimeS;    //时间，单位：秒
  unsigned int radarTimeNS;   //时间，单位：纳秒

  unsigned int lidarTimeS;    //时间，单位：秒
  unsigned int lidarTimeNS;   //时间，单位：纳秒

};

struct ODresults
{
  float confidence; 

  unsigned int track_id;
  unsigned int obj_type;
  std::string license_plate = "None";  

  double x_w;  // 局部坐标系下的 x 坐标 
  double y_w;  // 局部坐标系下的 y 坐标 
  double z_w;  // 局部坐标系下的 z 坐标 
  double yaw;  // 航向 (东北天坐标系)
  double vx;   // x轴速度 (东北天坐标系)
  double vy;   // y轴速度 (东北天坐标系)
  double vz;   // z轴速度 (东北天坐标系)
  double ax;   // x轴加速度 (东北天坐标系)
  double ay;   // y轴加速度 (东北天坐标系)
  double az;   // z轴加速度 (东北天坐标系)
  double wz;   // 角速度 (绕z轴)

  double lon;
  double lat;
  double alti; // 海拔高度
  
  double length;  // 目标 长度
  double width;   // 目标 宽度
  double height;  // 目标 高度

  unsigned long TimeS;    //时间，单位：秒
  unsigned long TimeNS;   //时间，单位：纳秒

  Point2F vertex[4];  //vertices of object
  double angle; // amgle in opencv rotated rect

  std::vector<BoundingBoxTrans> sensors_targets;
}; 



#endif
