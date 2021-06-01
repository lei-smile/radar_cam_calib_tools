#ifndef DATA_STRUCT_HPP
#define DATA_STRUCT_HPP

#include <string>
#include <vector>
#include <memory>
#include <opencv2/opencv.hpp>

#include "data_io.hpp" // TODO
//car:0 bus:1 track(truck):2 bikePerson:3 tricycle:4 person:5
static double LENGTH[6] = {4.4, 9.0, 10.0, 2.0, 1.6, 0.5};
static double WIDTH[6]  = {4.7, 2.4, 2.2, 1.5, 0.5, 0.5};
static double HEIGHT[6] = {1.5, 3.0, 2.7, 1.7, 1.5, 1.7};
//car:0 person:1
// static double LENGTH[6] = {4.4, 0.5, 9.0, 10.0, 2.0, 1.6};
// static double WIDTH[6]  = {4.7, 0.5, 2.4, 2.2, 1.5, 0.5};
// static double HEIGHT[6] = {1.5, 1.7, 3.0, 2.7, 1.7, 1.5};
struct bbox_t
{
  unsigned int x, y, w, h;
  float prob;
  unsigned int obj_id;
  unsigned int track_id;

  double length;
  double width;
  double height;

  unsigned long TimeS;    //时间，单位：秒
  unsigned long TimeNS;   //时间，单位：纳秒
};

struct image_t
{
  int h;
  int w;
  int c;
  char data[0];
};

// struct RadarData
// {
//   double x_radar;
//   double y_radar;
//   double r;
//   double angle;
//   double vx;
//   double vy;
//   double rcs;
//   int track_id;
//   int obj_id;
// };

// struct LidarData
// {
//   float x;
//   float y;
//   float z;
// };

struct CalibData
{
    double r;
    double theta;
    double lat;     //纬度
    double lon;     //经度
    double height;  //高度
    double x;
    double y;
    double z;
};


struct BoundingBox
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

  cv::RotatedRect rect;
  Point2F vertex[4];
  double angle;

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

  bool cerred = false;
  unsigned int TimeS;    //时间，单位：秒
  unsigned int TimeNS;   //时间，单位：纳秒

  unsigned int radarTimeS;    //时间，单位：秒
  unsigned int radarTimeNS;   //时间，单位：纳秒

  std::vector<RadarTrackings> radar_corres;
};  

// struct RadarTrackings
// {
//   std::string sensor_name;

//   std::vector<std::string> fusion_sensors;
//   std::vector<cv::Point2d> uv_projects; // 
  
//   float score; // score of the box;

//   unsigned int track_id;
//   unsigned int obj_id;

//   double r;
//   double angle;
//   double r_v;
//   double x_radar;
//   double y_radar;

//   double x_w;
//   double y_w;

//   double x_w_predict;
//   double y_w_predict;

//   bool cerred = false;
//   unsigned long TimeS;    //时间，单位：秒
//   unsigned long TimeNS;   //时间，单位：纳秒
// };

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


struct TargetGlobal
{
  unsigned int track_id;
  unsigned int obj_id;
  std::string license_plate = "None"; 
  cv::RotatedRect rect;
  Point2F vertex[4]; 
  double angle;
  double width;
  double length;
  double height;
  double h;

  unsigned long TimeS;    //时间，单位：秒
  unsigned long TimeNS;   //时间，单位：纳秒

  double x_pos;
  double y_pos;
  double yaw;
  double vx;
  double vy;
  double wz;
  double ax;
  double ay;

  int untracked_num;
  BoundingBox lidar_box;
  std::map<std::string, BoundingBox> sensors_target;
  std::vector<std::pair<std::string, BoundingBox>> sensors_targets;

  std::vector<BoundingBox> cam_tracks;
  unsigned int cam_track_id;
  bool cam_corred;
  std::vector<RadarTrackings> radar_tracks;
  bool radar_corred;
  unsigned int radar_track_id;

};

// struct SensorPairs
// {
//   std::string cam_name = "None";
//   std::string radar_name = "None";
//   std::string lidar_name = "None"; 
// };

// struct DataPairs
// {
//   SensorPairs sensor_pairs;

//   cv::Mat image; // TODO
//   std::vector<RadarData> radar_data; // TODO
//   std::vector<LidarData> lidar_data; // TODO

//   unsigned int imTimeS;    //时间，单位：秒 image
//   unsigned int imTimeNS;   //时间，单位：纳秒 image

//   unsigned int radarTimeS;    //时间，单位：秒 
//   unsigned int radarTimeNS;   //时间，单位：纳秒 

//   unsigned int lidarTimeS;    //时间，单位：秒 
//   unsigned int lidarTimeNS;   //时间，单位：纳秒 

//   image_t im;

// };

// struct MidResultsPairs
// {
//   SensorPairs sensor_pairs;

//   std::vector<BoundingBox> boxes_cam;
//   std::vector<RadarTrackings> targets_radar;
//   std::vector<LidarData> lidar_data;

//   unsigned int imTimeS;    //时间，单位：秒 image
//   unsigned int imTimeNS;   //时间，单位：纳秒 image

//   unsigned int radarTimeS;    //时间，单位：秒 
//   unsigned int radarTimeNS;   //时间，单位：纳秒 

//   unsigned int lidarTimeS;    //时间，单位：秒 
//   unsigned int lidarTimeNS;   //时间，单位：纳秒 

// };

// DataPairs db;
// memcopy(dist,db.im.data,  db.im.w*db.h*db.c) 

#endif
