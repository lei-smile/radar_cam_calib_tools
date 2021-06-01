#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

using namespace cv;
using namespace std;


std::vector<cv::Scalar>  colors{cv::Scalar(0,255,0),cv::Scalar(255,0,0),cv::Scalar(0,0,255),cv::Scalar(0,150,0),cv::Scalar(0,0,150)};

void readRadarData(const std::string& filename,
                                           std::vector<cv::Point2f>& radar_points)
{
  std::fstream fin(filename);
  if(!fin.is_open())
  {
    std::cout<<"fail to read radar file!!!"<<std::endl;
    return;
  }
  std::string line_str;
  while(std::getline(fin,line_str))
  {
    std::stringstream ss(line_str);
    std::string range_str, angle_str;
    std::getline(ss, range_str, ',');
    //std::cout<<"range_str::  "<<range_str<<std::endl;
    double range = std::atof(range_str.c_str());
    std::getline(ss, angle_str, ',');
    double angle = std::atof(angle_str.c_str());
    radar_points.emplace_back(cv::Point2f(range, angle));
  }

}

void readGPSData(const std::string& filename,
                                           std::vector<cv::Point3d>& gps_points)
{
  std::fstream fin(filename);
  if(!fin.is_open())
  {
    std::cout<<"fail to read gps file!!!"<<std::endl;
    return;
  }
  std::string line_str;
  while(std::getline(fin,line_str))
  {
    std::stringstream ss(line_str);
    std::string range_str, angle_str;
    std::getline(ss, range_str, ',');
    std::getline(ss, angle_str, ',');
    std::string lat_str, lon_str, height_str;
    std::getline(ss, lat_str, ',');
    std::getline(ss, lon_str, ',');
    std::getline(ss, height_str, ',');
    double lat = atof(lat_str.c_str());
    double lon = atof(lon_str.c_str());
    double height = atof(height_str.c_str());
    gps_points.emplace_back(cv::Point3d(lat, lon, height));
  }

}

void readGPSData_2(const std::string& filename,
                                           std::vector<cv::Point3d>& gps_points)
{
  std::fstream fin(filename);
  if(!fin.is_open())
  {
    std::cout<<"fail to read gps file!!!"<<std::endl;
    return;
  }
  std::string line_str;
  while(std::getline(fin,line_str))
  {
    std::stringstream ss(line_str);
    std::string other_str;
    std::getline(ss, other_str, ',');
    std::getline(ss, other_str, ',');
    std::getline(ss, other_str, ',');
    std::getline(ss, other_str, ',');
    std::getline(ss, other_str, ',');
    std::getline(ss, other_str, ',');
    std::getline(ss, other_str, ',');

    std::string lat_str, lon_str, height_str;
    std::getline(ss, lat_str, ',');
    std::getline(ss, lon_str, ',');
    std::getline(ss, height_str, ',');
    // double lat = std::stod(lat_str);
    // double lon = std::stod(lon_str);
    // double height = std::stod(height_str);
    double lat = atof(lat_str.c_str());
    double lon = atof(lon_str.c_str());
    double height = atof(height_str.c_str());
    gps_points.emplace_back(cv::Point3d(lat, lon, height));
  }

}

void readImageData(const std::string& filename,
                                           std::vector<cv::Point2f>& image_points)
{
  std::fstream fin(filename);
  if(!fin.is_open())
  {
    std::cout<<"fail to read image file!!!"<<std::endl;
    return;
  }
  std::string line_str;
  while(std::getline(fin,line_str))
  {
    std::stringstream ss(line_str);
    std::string u_str, v_str;
    std::getline(ss, u_str, ',');
    double u = std::stod(u_str);
    std::getline(ss, v_str, ',');
    double v = std::stod(v_str);
    image_points.emplace_back(cv::Point2f(u, v));
  }

}

int main()
{
  std::cout<<std::fixed<<std::setprecision(7)<<std::endl;
  std::string config_file = "../config/calib_config.yaml";
  
  cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
  if (!fsSettings.isOpened())
  {
    std::cerr << "ERROR:Wrong path to settings!!!!!!" << std::endl;
    return -1;
  }
  std::string calib_path;
  fsSettings["calib_path"] >> calib_path;
  std::string image_file;
  fsSettings["image_file"] >> image_file;
  image_file = calib_path + image_file;
  
  cv::Mat img_host1 = cv::imread(image_file);

//用标定数据反投影
//   std::string radar_points_file;
//   fsSettings["radar_file"] >> radar_points_file;
//   radar_points_file = calib_path + radar_points_file;
  //std::cout<<"radar_file:"<<radar_points_file<<std::endl;

  //用所有轨迹反投影
  std::string radar_calib_check_file;
  fsSettings["radar_calib_check_file"] >> radar_calib_check_file;
  radar_calib_check_file = calib_path + radar_calib_check_file;

    std::string points_reproject_file = radar_calib_check_file;

  std::string result_file;
  fsSettings["result_file"] >> result_file;
  result_file = calib_path + result_file;
  std::string radar_reproj_file;
  fsSettings["radar_reproject_file"] >> radar_reproj_file;
  radar_reproj_file = calib_path + radar_reproj_file;

  double lat0 = 0, lon0 = 0, height0 = 0;
  fsSettings["lat0"] >> lat0;
  fsSettings["lon0"] >>lon0;
  fsSettings["height0"] >> height0;
  const GeographicLib::Geocentric& earth = GeographicLib::Geocentric::WGS84();
  GeographicLib::LocalCartesian gps_origin(lat0, lon0, height0, earth); //以全局原点构建局部坐标系(东北天)
  

  cv::FileStorage calibSettings(result_file, cv::FileStorage::READ);

  cv::Mat UTM_to_cam;
  calibSettings["UTM2cam"] >> UTM_to_cam;
  std::cout << "UTM2cam \n" << UTM_to_cam << std::endl;
  
  double tx_radar = 0, ty_radar=0, h_radar = 0;
  double yaw_radar = 0, roll_radar =0,pitch_radar = 0;
  calibSettings["Tx"] >> tx_radar;
  printf("Tx: %lf\n", tx_radar);
  calibSettings["Ty"] >> ty_radar;
  printf("Ty:%lf\n", ty_radar);
  h_radar = 4.0;
  calibSettings["yaw"] >> yaw_radar;
  printf("yaw: %lf\n", yaw_radar);

  double delta_t=0;
  double delta_r = 0;
  std::vector<cv::Point2f> radar_points;
  readRadarData(points_reproject_file, radar_points);
  //std::string rtk_file = "/home/wulei/calib/static_calib/dataset/tj_84_2/rtk.txt";
  std::vector<cv::Point3d> gps_points;
  //readGPSData_2(rtk_file, gps_points);
  readGPSData(points_reproject_file, gps_points);

  //radar_points reproject on the image
  std::vector<cv::Point2f> radars_world;
  for(int i =0; i < radar_points.size();i++)
  {
    cv::Point2f radar_point = radar_points[i];
    double r = radar_point.x+ delta_r;
    double angle = radar_point.y*M_PI/180.0;   

    // angle = atan(tan(angle)/cos(pitch_radar));
    // angle = atan(tan(angle)/cos(roll_radar));                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         

    double d = sqrt(r*r - h_radar*h_radar);
    double x_radar = d*sin(angle);  
    double y_radar = d*cos(angle);

    double x_radar_w = x_radar*cos(yaw_radar) - y_radar*sin(yaw_radar) + tx_radar;  
    double y_radar_w = x_radar*sin(yaw_radar) + y_radar*cos(yaw_radar) + ty_radar;

    double x_w = x_radar_w;
    double y_w = y_radar_w;
    
    radars_world.push_back(cv::Point2f(x_w, y_w));
  }
  std::vector<cv::Point2f> radar_cam_uv;
  cv::perspectiveTransform(radars_world, radar_cam_uv, UTM_to_cam);
  
  for(int j =0; j < radar_cam_uv.size(); j++)
  {
    //std::cout<<"radar_Points::"<<radar_cam_uv[j].x<<"      "<<radar_cam_uv[j].y<<std::endl;
    if (radar_cam_uv[j].x < 5 || radar_cam_uv[j].x > 1915 || radar_cam_uv[j].y < 5 || radar_cam_uv[j].y > 1075) // TODO
    {
        // cam_uv[0].x = 1000;
        // cam_uv[0].y = 1000;
    }
    else
    {
        cv::Point radar_draw = cv::Point((int)radar_cam_uv[j].x , (int)radar_cam_uv[j].y);
        cv::circle(img_host1, radar_draw, 4, colors[1],-1);
        //cv::putText(img03, "Id:"+ std::to_string(p_radar.track_id), cv::Point((int)cam_uv[0].x, (int)cam_uv[0].y-5), 2, 1, colors[0], 2.0);
    }
  }

    //gps_points reproject on the image
    std::vector<cv::Point2d> gps_points_xy;
    std::vector<cv::Point2d> gps_points_uv;
    for(int i = 0; i < gps_points.size(); i++)
    {
      
      double x=0, y=0, z=0;
      gps_origin.Forward(gps_points[i].x, gps_points[i].y, gps_points[i].z, x, y, z);
      gps_points_xy.emplace_back(cv::Point2d(x, y));
     
      // std::cout<<"rtk_Points_xy::"<<gps_points_xy[i].x<<"      "<<gps_points_xy[i].y<<std::endl;
      
    }
  cv::perspectiveTransform(gps_points_xy, gps_points_uv, UTM_to_cam);
  for(int j =0; j < int(gps_points_uv.size()) - 1; j++)
  {
        if (gps_points_uv[j].x < 5 || gps_points_uv[j].x > 1915 || gps_points_uv[j].y < 5 || gps_points_uv[j].y > 1075) // TODO
        {
          // cam_uv[0].x = 1000;
          // cam_uv[0].y = 1000;
        }
      else
      {
          cv::Point2d gps_draw_now = cv::Point2d((int)gps_points_uv[j].x , (int)gps_points_uv[j].y);
          cv::Point2d gps_draw_next = cv::Point2d((int)gps_points_uv[j+1].x , (int)gps_points_uv[j+1].y);
          cv::circle(img_host1, gps_draw_now, 6, colors[0],-1);
          //cv::line(img_host1, gps_draw_now, gps_draw_next, cv::Scalar(0,0,255));
          //cv::putText(img03, "Id:"+ std::to_string(p_radar.track_id), cv::Point((int)cam_uv[0].x, (int)cam_uv[0].y-5), 2, 1, colors[0], 2.0);
      }
  }
  std::string points_on_road_file;
  fsSettings["points_on_road"] >> points_on_road_file;
  std::fstream fout(points_on_road_file, std::fstream::out);
  for(int k = 0; k < radars_world.size(); k++)
  {
    fout <<radars_world[k].x<<", "<<radars_world[k].y<<", "
             <<gps_points_xy[k].x<<", "<<gps_points_xy[k].y<<std::endl;
  }

  cv::imwrite(radar_reproj_file, img_host1);
  cv::resize(img_host1, img_host1, cv::Size( img_host1.cols*0.8, img_host1.rows*0.8), 0, 0, 0);
  cv::namedWindow("img_show",0);//创建窗口
  cv::imshow("img_show", img_host1);
  cv::waitKey();

}
