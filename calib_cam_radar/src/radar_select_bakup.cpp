#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include "data_struct.hpp"
#include "data_io.hpp"

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

using namespace cv;
using namespace std;

//#define SHOW_RADAR

std::vector<cv::Scalar>  colors{cv::Scalar(0,255,0),cv::Scalar(255,0,0),cv::Scalar(0,0,255),cv::Scalar(0,150,0),cv::Scalar(0,0,150)};

std::vector<std::pair<long double, std::vector<RadarData>>> radar_data_times;

struct RadarDataTime
{
  long double time;
  std::vector<RadarData> radar_datas;
  unsigned int TimeS;    //时间，单位：秒
  unsigned int TimeNS;   //时间，单位：纳秒
};
struct ImageDataTime
{
  std::string im_path;
  long double time;
  unsigned int TimeS;    //时间，单位：秒
  unsigned int TimeNS;   //时间，单位：纳秒
};

std::vector<RadarDataTime> radar_data_times_tj;
std::vector<ImageDataTime> image_data_times_tj;
std::vector<RadarDataTime> radar_data_times_wh01;
std::vector<ImageDataTime> image_data_times_wh01;
std::vector<RadarDataTime> radar_data_times_wh02;
std::vector<ImageDataTime> image_data_times_wh02;
std::vector<RadarDataTime> radar_data_times_wh03;
std::vector<ImageDataTime> image_data_times_wh03;

double lat0 = 0, lon0 = 0, height0 = 0;



bool readRadarData(std::string fileName, RadarDataTime& points_radar, char separator = ',')
{
  cv::Size im_size2(1000, 1000);
  cv::Mat img2(im_size2, CV_8UC3, CV_RGB(200,200,200));
  std::ifstream ifile(fileName);
  if (!ifile.is_open())
  {
    std::cerr << "file can't opened  !!!!!!!!!" <<std::endl;
    return false;
  }

  std::string lineStr;

  while (std::getline(ifile, lineStr))
  {
    std::stringstream ss(lineStr);
    std::string str;
    RadarData p_radar;
   
    //timestame:1611911455.334571650, objid:1, type:0, xpos:0.0ypos:0.0,xspeed:9.03, yspeed:-6.51,distense:98,angle:-42.7
    //timestame:1614673256.230000000, objid:0, type:0, xpos:0.0ypos:0.0,xspeed:13.14,yspeed:-0.90,angle:-2.21,distense:195.400,rcs:15.000000
    std::getline(ss, str, separator); //timestame:1614673256.230000000
      std::stringstream ss0(str);
      std::string str0;
      std::getline(ss0, str0, ':');  //timestame
      std::getline(ss0, str0, ':'); // 1614673256.230000000
      std::stringstream ss00(str0);
      std::string str00;
      std::getline(ss00, str00, '.'); //1614673256
        unsigned int a_s = strtoul(str00.c_str(),NULL,10);
        // std::cout <<"unsigned int a_s  "<< a_s << std::endl;
      std::getline(ss00, str00, '.'); //230000000
        unsigned int c_ns = strtoul(str00.c_str(),NULL,10);
        // std::cout <<"unsigned int c_ns  "<< c_ns << std::endl;
        points_radar.TimeS = a_s;
        points_radar.TimeNS = c_ns;
        long double time_second = (long double)a_s;
        long double time_ns = (long double)c_ns/1000000000.0;
        points_radar.time = time_second + time_ns;
        
    std::getline(ss, str, separator); //objId:0
      std::stringstream ss1(str);
      std::string str1;
      std::getline(ss1, str1, ':');   //objId
      std::getline(ss1, str1, ':'); p_radar.track_id = atoi(str1.c_str()); // 0

    std::getline(ss, str, separator); //type:0
    std::getline(ss, str, separator);
    std::getline(ss, str, separator); //xpos:0.0ypos:0.0

    std::getline(ss, str, separator); //xspeed:13.14
      std::stringstream ss2(str);
      std::string str2;
      std::getline(ss2, str2, ':');   //xspeed
      std::getline(ss2, str2, ':'); p_radar.vx = atof(str2.c_str()); // 13.14

    std::getline(ss, str, separator); //yspeed:-0.90

    std::getline(ss, str, separator); //angle:-2.21
      std::stringstream ss3(str);
      std::string str3;
      std::getline(ss3, str3, ':');  // angle
      std::getline(ss3, str3, ':'); p_radar.angle = atof(str3.c_str()); //-2.21


    std::getline(ss, str, separator); //distense:195.400
      std::stringstream ss4(str);
      std::string str4;
      std::getline(ss4, str4, ':');  //distense
      std::getline(ss4, str4, ':'); p_radar.r = atof(str4.c_str()); // 195.400

    double theta = p_radar.angle*M_PI/180.0;
    p_radar.x_radar = 1.0*p_radar.r*sin(theta);
    p_radar.y_radar = p_radar.r*cos(theta);

    printf("FUNC:%s LINE:%d\n",__FUNCTION__,__LINE__);
    points_radar.radar_datas.push_back(p_radar);
    
  } 
  ifile.close();
  return true;
}


bool ReadImageTime(std::string path, cv::Mat& img01, unsigned int& time_s, unsigned int& time_ns, long double& time_ld)
{
    img01 = cv::imread(path);
    std::cout << "img_path " << path << std::endl; 
    string im_t_path01 = path;
    // string im_t_string01 = im_t_path01.substr(42,19);
    //../data/tianjin1.31/car_image/1_1612072412482056000.jpg
    string im_t_string01 = im_t_path01.substr(6,19);
    
    string im01_s = im_t_string01.substr(0,10);
   
    string im01_us = im_t_string01.substr(10,6);
    
    string im01_ns = im_t_string01.substr(10,9);
    
    unsigned int a01_s = strtoul(im01_s.c_str(),NULL,10);
    
    unsigned int b01_us = strtoul(im01_us.c_str(),NULL,10);
   
    unsigned int c01_ns = strtoul(im01_ns.c_str(),NULL,10);
    

    long double time_second = (long double)a01_s;
    long double time_us = (long double)b01_us;
    long double im_time = time_second + time_us/1000000.0;

    time_s = a01_s;
    time_ns = c01_ns;
    time_ld = im_time;
    return true;
}

void readGPSData(std::string fileName, std::vector<GPSinsData>& gps_datas, char separator)
{
  std::ifstream ifile(fileName);
  std::string lineStr;
  if (!ifile.is_open())
  {
    std::cerr << "RTK ins file cannot openned " << std::endl;
    return;
  }
  
  static const GeographicLib::Geocentric& earth = GeographicLib::Geocentric::WGS84();
  GeographicLib::LocalCartesian gps_trans(lat0, lon0, height0, earth); //以原点构建局部坐标系(东北天)

  while (std::getline(ifile, lineStr))
  {
    std::stringstream ss(lineStr);
    std::string gps_time_str;
    std::string str;
    GPSinsData dataInfo;

    std::getline(ss, gps_time_str, separator);  // 1610171708880000000
    
    string gps_time_s = gps_time_str.substr(0,10);
   
    string gps_time_us = gps_time_str.substr(10,6);
    
    unsigned int time_s = strtoul(gps_time_s.c_str(),NULL,10);
    
    unsigned int time_us = strtoul(gps_time_us.c_str(),NULL,10);
    
    long double time_second = (long double)time_s;
    long double time_usecond = (long double)time_us;
    long double gps_time = time_second + time_usecond/1000000.0;

    dataInfo.time_stamp = gps_time;
    

    std::getline(ss, str, separator); // $GPFPD
    std::getline(ss, str, separator); // 2139
    std::getline(ss, str, separator); // 5539726.860
    std::getline(ss, str, separator); // 177.532
    std::getline(ss, str, separator); // 3.274
    std::getline(ss, str, separator); // 0.923
    std::getline(ss, str, separator); dataInfo.lat = atof(str.c_str());
    std::getline(ss, str, separator); dataInfo.lon = atof(str.c_str());
    std::getline(ss, str, separator); dataInfo.height = atof(str.c_str());

    std::getline(ss, str, separator); string lat_s = str;
    std::getline(ss, str, separator); string lon_s = str;
    std::getline(ss, str, separator); string height_s = str;

    //gps_trans.Forward(dataInfo.lat, dataInfo.lon, dataInfo.height, dataInfo.x, dataInfo.y, dataInfo.z);
    
    gps_datas.push_back(dataInfo);
  }

}

int main(int argc, char** argv)
{
  int radar_selectedid = std::stoi(argv[1]);
  std::cout<<"radar_selectedid:  "<<radar_selectedid<<std::endl;
  std::string config_file = "./sensors_calib.yml";  //sensors_calib.yml
  std::fstream file_result;
  file_result.open("./radar_points.txt", fstream::out);
  if(!file_result.is_open())
  {
    std::cout<<"Fail to open saved file"<<std::endl;
    return 0;
  }
  file_result<<fixed<<setprecision(7);
  
  std::vector<cv::String> img_files;
  std::vector<cv::String> radar_files;
  cv::glob("pic/*.jpg", img_files);
  cv::glob("radar/*.txt", radar_files);

  for (int i = 0; i < radar_files.size(); ++i)
  {
      RadarDataTime points_radar;
      bool flag = readRadarData(radar_files[i], points_radar, ',');
      radar_data_times_wh03.push_back(points_radar);
  }
  std::string rtk_file = "./rtk.txt";
  std::vector<GPSinsData> gps_data;
  readGPSData(rtk_file, gps_data, ',');

  double delta_t = 0.0;
  double delta_r = 0.0;
  bool first = true;
  for (int i = 0; i < img_files.size(); i+=3)
  {
    printf("FUNCTION:%s\n", __FUNCTION__);
    char key = cv::waitKey(200);
    if(key == ' ')
    {
      std::cout << " pause !!!!!!!!!!!!!!!!!!!!!!!!!!  i " << i << std::endl;
      char key02 = 'a';
      while(key02 != ' ')
      {
        key02 = cv::waitKey(200);
      }
    }
    unsigned int time03_s,time03_ns;
    long double time03_ld;
    cv::Mat img03;
    bool flag003 = ReadImageTime(img_files[i],img03,time03_s, time03_ns, time03_ld);
    std::cout<<"img_files[i]:"<<img_files[i]<<std::endl;
    if (!flag003)
    {
      std::cout<<"Fail to read image time!!!!"<<std::endl;
      continue;
    }
       
    std::vector<RadarData> points_radar03;
    unsigned int radar03_time_s;
    unsigned int radar03_time_ns;

    
    for (int j = 0; j < radar_data_times_wh03.size()-1; ++j)
    {
        if ((radar_data_times_wh03[j].time - 0.0) < time03_ld + delta_t && (radar_data_times_wh03[j+1].time + 0.0) > time03_ld + delta_t)
        {
          points_radar03 = radar_data_times_wh03[j].radar_datas;
          radar03_time_s = radar_data_times_wh03[j].TimeS;
          radar03_time_ns = radar_data_times_wh03[j].TimeNS;
          break;
        }
    } 

    RadarData selected_radar_point;
    GPSinsData selected_gps_point;
    bool radar_selected_flag = false;
    bool gps_selected_flag = false;
    for(int k = 0; k < points_radar03.size(); k++)
    {
      std::cout<<"track_id:::"<<points_radar03[k].track_id<<std::endl;
      if(points_radar03[k].track_id ==radar_selectedid)
      {
        //file_result << points_radar03[k].r<<", "<<points_radar03[k].angle<<std::endl;
      //   std::cout << points_radar03[k].r<<", "<<points_radar03[k].angle<<std::endl;
        radar_selected_flag = true;
        selected_radar_point = points_radar03[k];
      }
    }

    for(int j = 0; j < gps_data.size(); j+=4)
    {
      //std::cout<<"gps_time:  "<<gps_data[j].time_stamp<<std::endl;
      //std::cout<<"img_time:"<<time03_ld<<std::endl;
      if ((gps_data[j].time_stamp - 0.0) < time03_ld + delta_t && (gps_data[j + 4].time_stamp + 0.0) > time03_ld + delta_t)
      {
        selected_gps_point = gps_data[j];
        gps_selected_flag = true;
        //file_result<<selected_gps_point.lat<<", "<<selected_gps_point.lon<<", "<<selected_gps_point.height<<std::endl;
        //std::cout<<selected_gps_point.lat<<", "<<selected_gps_point.lon<<", "<<selected_gps_point.height<<std::endl;
      }
    }
    if(radar_selected_flag && gps_selected_flag)
    {
      file_result << selected_radar_point.r <<", "<<selected_radar_point.angle<<", "
                           << selected_gps_point.lat<<", "<<selected_gps_point.lon<<", "
                           << selected_gps_point.height<<std::endl;
    }

#ifdef SHOW_RADAR

    cv::Size im_size02(1000, 1000);
    cv::Mat img02(im_size02, CV_8UC3, CV_RGB(200,200,200));
    for (int k =0; k < points_radar03.size(); ++k)
    {
      RadarData p_radar = points_radar03[k];
      std::cout<<"radar_x:  "<<p_radar.x_radar<<std::endl;
      std::cout<<"radar_y:  "<<p_radar.y_radar<<std::endl;
      std::cout<<"radar_r:  "<<p_radar.r<<std::endl;
      std::cout<<"radar_angle:  "<<p_radar.angle<<std::endl;

      int u = 5.0*(p_radar.x_radar) + 500;
      int v = 1000 - 5.0*p_radar.y_radar;
      if(u < 30)
        u = 30;
      if(u > 970)
        u = 970; 
      if(v < 30)
        v = 30;
      if(v > 970)
        v = 970; 

      cv::Point p_draw = cv::Point(u, v);
      cv::circle(img02, p_draw, 3, colors[0],-1);
      cv::putText(img02, "Id:"+ std::to_string(p_radar.track_id), cv::Point(p_draw.x, p_draw.y-5), 1, 1, colors[4], 1.0);

      cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
      if (!fsSettings.isOpened())
      {
        std::cerr << "ERROR:Wrong path to settings!!!!!!" << std::endl;
        return -1;
      }
      cv::Mat UTM_to_cam;
      fsSettings["UTM_to_cam_yushi_wh02"] >> UTM_to_cam;
      //std::cout << "UTM_to_cam_yushi_wh02 \n" << UTM_to_cam << std::endl;
     
      double tx_radar = 0, ty_radar=0, h_radar = 0;
      double yaw_radar = 0, roll_radar =0,pitch_radar = 0;

      fsSettings["Tx_radar_sunyan_wh02"] >> tx_radar;
      fsSettings["Ty_radar_sunyan_wh02"] >> ty_radar;
      fsSettings["H_radar_sunyan_wh02"] >> h_radar;
      fsSettings["YAW_radar_sunyan_wh02"] >> yaw_radar;

      double r = p_radar.r + delta_r;
      double angle = p_radar.angle*M_PI/180.0;   

      angle = atan(tan(angle)/cos(pitch_radar));
      angle = atan(tan(angle)/cos(roll_radar));                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         

      double d = sqrt(r*r - h_radar*h_radar);
      double x_radar = d*sin(angle);  
      double y_radar = d*cos(angle);

      double x_radar_w = x_radar*cos(yaw_radar) - y_radar*sin(yaw_radar) + tx_radar;  
      double y_radar_w = x_radar*sin(yaw_radar) + y_radar*cos(yaw_radar) + ty_radar;

      double x_w = x_radar_w;
      double y_w = y_radar_w;

        cv::Point2f p2f;
        std::vector<cv::Point2f> cam_uv;
        p2f.x = x_w;
        p2f.y = y_w;
        std::vector<cv::Point2f > radar_w;
        radar_w.push_back(p2f);

        cv::perspectiveTransform(radar_w, cam_uv, UTM_to_cam);
        if (cam_uv[0].x < 5 || cam_uv[0].x > 1915 && cam_uv[0].y < 5 && cam_uv[0].y > 1075) // TODO
        {
            // cam_uv[0].x = 1000;
            // cam_uv[0].y = 1000;
        }
        else
        {
            cv::Point p_draw03 = cv::Point((int)cam_uv[0].x , (int)cam_uv[0].y);
            std::cout<<"Points::"<<cam_uv[0].x<<cam_uv[0].y<<std::endl;
            cv::circle(img03, p_draw03, 6, colors[2],-1);
            cv::putText(img03, "Id:"+ std::to_string(p_radar.track_id), cv::Point((int)cam_uv[0].x, (int)cam_uv[0].y-5), 2, 1, colors[0], 2.0);
        }


    }
    cv::namedWindow("radar_raw",0);//创建窗口
    cv::imshow("radar_raw",img02);
#endif 

    cv::resize(img03, img03, cv::Size( img03.cols*0.8, img03.rows*0.8), 0, 0, 0);
    cv::namedWindow("img03",0);//创建窗口
    cv::imshow("img03",img03);
    cv::waitKey(1);
    while(first)
    {
      char key = cv::waitKey(1000);
      if(key == ' ')
        first = false;
    }

  }

return 0;
}
