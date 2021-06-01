#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <string>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include "data_struct.hpp"

using namespace cv;
using namespace std;

void readCalibData(std::string fileName, std::vector<CalibData>& check_calib_datas, char separator = ',')
{
  std::fstream file_data(fileName);
  std::string line_str;
  while(std::getline(file_data, line_str))
  {
    CalibData data_read;
    std::stringstream ss(line_str);
    std::string factor_str;
    std::getline(ss, factor_str, separator);
    data_read.r = atof(factor_str.c_str());
    std::getline(ss, factor_str, separator);
    data_read.theta = atof(factor_str.c_str());

    std::getline(ss, factor_str, separator);
    data_read.lat = atof(factor_str.c_str());
    std::cout<<std::fixed<<std::setprecision(8);
    
    std::getline(ss, factor_str, separator);
    data_read.lon = atof(factor_str.c_str());
    std::getline(ss, factor_str, separator);
    data_read.height = atof(factor_str.c_str());
    // std::cout<<"lat::::"<<data_read.lat<<std::endl;
    // std::cout<<"lon::::"<<data_read.lon<<std::endl;
    // std::cout<<"height::::"<<data_read.height<<std::endl;
    check_calib_datas.emplace_back(data_read);
  }

}


//将radar的r和thata转换为x、y
void trans_radar_xy(double Tx, double Ty, double h, double yaw, 
                                           double radar_r, double radar_angle, double& radar_x_inworld, double& radar_y_inworld)
{
        double d = sqrt(radar_r * radar_r - h*h);
        double x_radar = d*sin(radar_angle *M_PI / 180.0);  
        double y_radar = d*cos(radar_angle * M_PI / 180.0);

        radar_x_inworld = x_radar*cos(yaw) - y_radar*sin(yaw) + Tx;  
        radar_y_inworld = x_radar*sin(yaw) + y_radar*cos(yaw) + Ty;
}



int main()
{
        std::string config_file = "../config/calib_config.yaml";
        cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
        if (!fsSettings.isOpened())
        {
                std::cerr << "ERROR:Wrong path to settings!!!!!!" << std::endl;
                return -1;
        }
        int mec_num = 0;
        int data_num = 0;
        fsSettings["mec_num"] >>mec_num;    //路口的总设备数
        fsSettings["data_num"] >> data_num;   //当前处理的radar数据采集于哪一台设备，在衡量radar.r与rtk直线距离时有用

        double lat0 = 0.0, lon0 = 0.0, height0 = 0.0;  //  全局原点 
        fsSettings["lat0"] >> lat0;
        fsSettings["lon0"] >> lon0;
        fsSettings["height0"] >> height0;
        static const GeographicLib::Geocentric& earth = GeographicLib::Geocentric::WGS84();
        GeographicLib::LocalCartesian global_gps_trans(lat0, lon0, height0, earth); //以原点构建局部坐标系(东北天)

        cv::Mat img_birdview(cv::Size(1000, 1000), CV_8UC3,cv::Scalar(255,255,255));
        std::vector<cv::Point2f> vertex_rtk;
        for(int i = 0; i < mec_num; i++)
        {
                double lat = 0.0, lon = 0.0;
                std::string lat_str = "lat" + std::to_string(i+1); 
                std::string lon_str = "lon" + std::to_string(i+1);
                fsSettings[lat_str] >> lat;
                fsSettings[lon_str] >>lon;
                vertex_rtk.push_back(cv::Point2f(lat, lon));
        }
        
        std::vector<cv::Point2f> vertex_xy;
        for(int i = 0 ; i < mec_num; i++)
        {
                double x=0.0, y=0.0, z=0.0; 
                global_gps_trans.Forward(vertex_rtk[i].x, vertex_rtk[i].y, 0.0, x, y, z);
                vertex_xy.push_back(cv::Point2f(x, y));
                std::cout<<"radar_"<<i+1<<"_position:  "<<x<<"     "<<y<<std::endl;
                
        }
        for(int i = 0; i < mec_num; i++)
        {
                cv::Point2f p_draw(vertex_xy[i].x * 4 + 500, vertex_xy[i].y * 4 + 500);
                cv::circle(img_birdview,p_draw, 5, cv::Scalar(0, 255, 0), -1);
                cv::putText(img_birdview, cv::String(std::to_string(i + 1)),p_draw,0,1, cv::Scalar(0,255,0) );
                cv::Point2f p_1 = p_draw;
                cv::Point2f p_2(vertex_xy[(i+1)%4].x * 4+ 500, vertex_xy[(i+1)%mec_num].y * 4 + 500);
                cv::line(img_birdview,p_1, p_2, cv::Scalar(255, 0, 0), 2);
        }
        
        //读取rtk和radar数据
        std::string calib_path;
        fsSettings["calib_path"] >> calib_path;
        std::string radar_calib_check_file;
        fsSettings["radar_calib_check_file"] >> radar_calib_check_file;
        radar_calib_check_file = calib_path + radar_calib_check_file;
        std::string radar_birdview_file;
        fsSettings["radar_birdview_file"] >> radar_birdview_file;
        radar_birdview_file = calib_path + radar_birdview_file;

        std::string calib_result_file;
        fsSettings["result_file"] >>calib_result_file;
        calib_result_file = calib_path + calib_result_file;

        cv::FileStorage calibResult(calib_result_file, cv::FileStorage::READ);
        if (!calibResult.isOpened())
        {
                std::cerr << "ERROR:Wrong path to calibResult!!!!!!" << std::endl;
                return -1;
        }
        double Tx = 0.0;           
        double Ty = 0.0;
        double h = 4.0;
        double yaw_radar = 0.0;
        calibResult["Tx"] >> Tx;
        calibResult["Ty"] >>Ty;
        calibResult["H_radar"] >>h;
        calibResult["yaw"] >>yaw_radar;
        std::cout<<"yaw::  "<<yaw_radar<<std::endl;

        std::vector<CalibData> check_calib_datas;
        readCalibData(radar_calib_check_file, check_calib_datas);
        std::cout<<"check_calib_datas.size():"<<check_calib_datas.size()<<std::endl;
        double x_error = 0.0;
        double y_error = 0.0;
        double distance_error = 0.0;
        for(int i = 0; i < check_calib_datas.size(); i++)
        {
            double rtk_lat = check_calib_datas[i].lat;
            double rtk_lon = check_calib_datas[i].lon;
            double rtk_height = check_calib_datas[i].height;
            double rtk_x=0, rtk_y=0, rtk_z=0;
            global_gps_trans.Forward(rtk_lat, rtk_lon, rtk_height, rtk_x, rtk_y, rtk_z);
            cv::Point2d rtk_point(rtk_x*4+500, rtk_y*4+500);
            cv::circle(img_birdview, rtk_point, 3, cv::Scalar(0, 255, 0), -1);

            double radar_r = check_calib_datas[i].r;
            double radar_angle = check_calib_datas[i].theta;
            double radar_x_inworld=0, radar_y_inworld=0;
            trans_radar_xy(Tx, Ty, h, yaw_radar, radar_r, radar_angle, radar_x_inworld, radar_y_inworld);
            cv::Point2d radar_point(radar_x_inworld*4+500, radar_y_inworld*4+500);
            cv::circle(img_birdview, radar_point, 3, cv::Scalar(0,0,255), -1);

            x_error += fabs(rtk_x - radar_x_inworld);
            y_error += fabs(rtk_y - radar_y_inworld);
            distance_error += sqrt(pow(rtk_x-radar_x_inworld, 2) + pow(rtk_y-radar_y_inworld, 2));
        }
        x_error = x_error / double(check_calib_datas.size());
        y_error = y_error / double(check_calib_datas.size());
        distance_error = distance_error / double(check_calib_datas.size());

        std::cout<<"avg_x_error:  "<<x_error<<std::endl;
        std::cout<<"avg_y_error:  "<<y_error<<std::endl;
        std::cout<<"avg_distance_error:  "<<distance_error<<std::endl;
        cv::FileStorage calibError(calib_result_file, cv::FileStorage::APPEND);
        calibError<<"radar_rtk_x_error"<<x_error;
        calibError<<"radar_rtk_y_error"<<y_error;
        calibError<<"radar_rtk_distance_error"<<distance_error;
        cv::imshow("img_birdview", img_birdview);
        cv::imwrite(radar_birdview_file, img_birdview);
        cv::waitKey();


}

