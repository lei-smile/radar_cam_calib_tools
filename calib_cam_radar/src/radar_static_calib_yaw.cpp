#include <iostream>
#include <exception>
#include <cmath>
#include <fstream>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <stdio.h>
#include <iomanip>

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include <opencv2/opencv.hpp>

#include "data_struct.hpp"


bool ReadConfigfile(const std::string& config_fileName,std::string& calib_data_file,
                                            std::string& result_file,
                                            double& lat0, double& lon0, double& height0,
                                            double& lat_radar, double& lon_radar, double& height_radar)
{
  cv::FileStorage config_read(config_fileName, cv::FileStorage::READ);
  if(!config_read.isOpened())
  {
    std::cerr<<"Read the calib config file failed！"<<std::endl;
    return false;
  }
  std::string calib_path;
  config_read["lat0"] >> lat0;
  config_read["lon0"] >> lon0;
  config_read["height0"] >> height0;
  config_read["calib_path"] >> calib_path; 
  config_read["radar_file"] >> calib_data_file;
  calib_data_file = calib_path + calib_data_file;
  config_read["result_file"] >> result_file;
  result_file = calib_path + result_file;

  int data_num;
  config_read["data_num"] >> data_num;
  std::string lat_radar_num = "lat"+std::to_string(data_num); 
  config_read[lat_radar_num] >> lat_radar;        //获取当前标定的毫米波雷达的经纬度信息
  std::string lon_radar_num = "lon"+std::to_string(data_num);
  config_read[lon_radar_num] >> lon_radar;
  std::string height_radar_num = "height"+std::to_string(data_num);
  config_read[height_radar_num] >> height_radar;

  std::cout<<lat0<<" "<<lon0<<" "<<height0<<std::endl;
  std::cout<<"radar_file::"<<calib_data_file<<std::endl;

  return true;
}

void readCalibData(std::string fileName, std::vector<CalibData>& calib_datas, char separator = ',')
{
  std::fstream file_data(fileName);
  std::string line_str;
  while(std::getline(file_data, line_str))
  {
    CalibData data_read;
    std::stringstream ss(line_str);
    std::string factor_str;
    std::getline(ss, factor_str, separator);
    data_read.r = std::stod(factor_str);
    std::getline(ss, factor_str, separator);
    data_read.theta = std::stod(factor_str);

    std::getline(ss, factor_str, separator);
    data_read.lat = atof(factor_str.c_str());
    std::cout<<std::fixed<<std::setprecision(8);
    
    std::getline(ss, factor_str, separator);
    data_read.lon = atof(factor_str.c_str());
    std::getline(ss, factor_str, separator);
    data_read.height = atof(factor_str.c_str());
    std::cout<<"lat::::"<<data_read.lat<<std::endl;
    std::cout<<"lon::::"<<data_read.lon<<std::endl;
    std::cout<<"height::::"<<data_read.height<<std::endl;
    calib_datas.emplace_back(data_read);
  }

}


struct RadarGPSFactorYTH
{
  RadarGPSFactorYTH(CalibData data_calib) 
      : data_calib_(data_calib)
  {
      std::cout<<"data_calib:::::"<<data_calib.x<<"    "<<data_calib.y<<"  "
                            <<data_calib.z<<"   "<<data_calib.r<<"   "<<data_calib.theta<<std::endl;

  }
  template <typename T>
  bool operator()(const T *params, T *residual) const
  {

    // T x_w = T(data_calib_.x);//- data_calib_.v_x*params[1];
    // T y_w = T(data_calib_.y);//- data_calib_.v_y*params[1];
    // T x_project = x_w*cos(params[0]) - y_w*sin(params[0]);
    // T y_project = x_w*sin(params[0]) + y_w*cos(params[0]);

    // T theta = T(data_calib_.theta) / T(180.0) * T(3.1415926);
    // // theta = asin(sin(theta) * params[1]);
    // T x_radar = data_calib_.r * sin(theta) + params[0];
    // T y_radar = sqrt(pow(data_calib_.r * cos(theta), 2) - params[3]*params[3]) + params[1];    //法一
    // // T y_radar = data_calib_.r * cos(theta) + params[1];                                                       //法二

    // residual[0] = x_radar - x_project;
    // residual[1] = y_radar - y_project;

    //法三
    T range = T(data_calib_.r);
    T angle = T (data_calib_.theta) / 180.0 * 3.1415926;
    T d = sqrt(range*range - 4.0*4.0);
    T x_radar = d*sin(angle);
    T y_radar = d*cos(angle);
    T x_radar_w = x_radar * cos(params[0]) - y_radar * sin(params[0]);
    T y_radar_w = x_radar * sin(params[0]) + y_radar * cos(params[0]);
    T x = T(data_calib_.x); //+ params[1];
    T y = T(data_calib_.y);// + params[2];
    residual[0] = x_radar_w - x;
    residual[1] = y_radar_w - y;

    return true;
  }

  static ceres::CostFunction *Create(const CalibData data_calib)
  {
    //2是残差residual的维度， 3是优化参数params的维度
    return (new ceres::AutoDiffCostFunction<RadarGPSFactorYTH, 2, 4>(new RadarGPSFactorYTH(data_calib)));
  }

  CalibData data_calib_;

};

void calibOptimizeRadar(std::vector<CalibData> points_calib, 
                                                     std::string& result_file)
{
  double radar_params[4] = {0.0, 0.0, 0.0, 4.0}; //优化参数三个：毫米波安装航向角yaw、
                                                                                              // 毫米波原点偏差delta_x、delta_y
  
  ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
  ceres::Problem::Options problem_options;

  ceres::Problem problem(problem_options);
  problem.AddParameterBlock(radar_params, 4);     //radar_params的维度

  std::cout << "value after optimizationA:"<<std::endl;
  for (int i = 0; i < points_calib.size(); ++i)
  {
    ceres::CostFunction *cost_function = RadarGPSFactorYTH::Create(points_calib[i]);
    problem.AddResidualBlock(cost_function, loss_function, radar_params);
  }
  std::cout << "value after optimizationB:"<<std::endl;
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.max_num_iterations = 100;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << "value after optimization:"<<std::endl;
  std::cout << "params angle:: " << radar_params[0] / 3.14159 * 180.0<<std::endl;
  std::cout<<"params theta::"<<radar_params[0]<<std::endl;
  std::cout<<"params_delta_x: "<<radar_params[1]<<"     params_delta_y:"<<radar_params[2]<<std::endl;
  std::cout<<"params_delta_h: "<<radar_params[3]<<std::endl;
  // std::string result_file = "../result.txt";
  std::ofstream data_save(result_file, std::ofstream::app);
  data_save <<"yaw:  "<<radar_params[0] <<std::endl;
}


struct CURVE_FITTING_COST
{
  CURVE_FITTING_COST(double x, double y, double range, double theta):x_(x), y_(y), range_(range), theta_(theta)
  {
    // double alpha = - 2.0 / 180.0 * M_PI;
    // x_ = cos(alpha) * x - sin(alpha) * y;
    // y_ = sin(alpha) * x + cos(alpha) * y;
     x_ = x;
     y_ = y;
    range_ = range;
    theta_ = theta / 180 * M_PI;

  }
  template <typename T>
  bool operator()(const T* const para,T* residual)const        //para[2]={H-h, theta_offset}
  {
    // residual[0] = T(x_) - T(range_) * T(cos(T(theta_)) + para[2]) * para[1] / para[0];
    // residual[0] = T(y_) - sqrt(pow(T(range_ * cos(theta_ + para[1])), 2) - pow((para[0]), 2));
    //residual[0] = T(y_) - T(range_ * cos(theta_ + para[0]));


    T x_project = x_*cos(para[0]) - y_*sin(para[0]);
    T y_project = x_*sin(para[0]) + y_*cos(para[0]);
    residual[0] = T(y_project) - sqrt(T(pow(range_ * cos(theta_), 2) ) - pow(T(6.0), 2)); //- T(1.0);
    residual[1] = T(x_project) - T(range_) * T(sin(T(theta_)));
    return true;
  }
  double x_, y_, range_, theta_;
};

void optimize(std::vector<CalibData> calibDataVec)
{
  double para[1] = {1.3};  // 初始化para[2] = {H-h, theta_offset};
  ceres::Problem problem;
  // for(int i=0;i<calibDataVec.size(); i++)
  for(int i =0 ; i < calibDataVec.size() ; i ++)
  {
    problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CURVE_FITTING_COST,2,1>(
        new CURVE_FITTING_COST(calibDataVec[i].x, calibDataVec[i].y, calibDataVec[i].r, calibDataVec[i].theta)
      ),
      nullptr,
      para
    );
   
  }
  std::cout<<"333333"<<std::endl;
//配置求解器并求解，输出结果
  ceres::Solver::Options options;
  options.linear_solver_type=ceres::DENSE_QR;
  options.minimizer_progress_to_stdout=true;
  ceres::Solver::Summary summary;
  ceres::Solve(options,&problem,&summary);
  std::cout<<"theta= "<<para[0] / M_PI * 180.0<<std::endl;
}

int main(int argc, char *argv[])
{
  std::string config_file = "../config/calib_config.yaml";
  std::string data_path;  //calib_data_file
  std::string result_path;
  double lat0=0, lon0=0, h0=0;
  double lat_radar = 0, lon_radar = 0, h_radar = 0;
  ReadConfigfile(config_file, data_path, result_path, lat0, lon0, h0, lat_radar, lon_radar, h_radar);

  const GeographicLib::Geocentric& earth = GeographicLib::Geocentric::WGS84();
  GeographicLib::LocalCartesian gps_origin(lat0, lon0, h0, earth); //以全局原点构建局部坐标系(东北天)
  GeographicLib::LocalCartesian gps_radar(lat_radar, lon_radar, h_radar, earth); //以毫米波雷达坐标为原点构建局部坐标系(东北天)

  std::vector<CalibData> calib_datas;
  readCalibData(data_path, calib_datas);
  for(int i = 0; i < calib_datas.size(); i++)
  {
    double lat = calib_datas[i].lat;
    double lon = calib_datas[i].lon;
    double height = calib_datas[i].height;
    double x=0, y=0, z=0;
    gps_radar.Forward(lat, lon, height, x, y, z);
    calib_datas[i].x = x;
    calib_datas[i].y = y;
    calib_datas[i].z = z;
    std::cout<<"x:"<<x<<"      y:"<<y<<"            z"<<z<<std::endl;
  }
  calibOptimizeRadar(calib_datas, result_path);
  
  double radar_x = 0, radar_y = 0, radar_z = 0;
  gps_origin.Forward(lat_radar, lon_radar, h_radar, radar_x, radar_y, radar_z);
  std::ofstream data_save(result_path, std::ofstream::app);
  data_save << "Tx: "<<radar_x<<std::endl;
  data_save << "Ty: "<<radar_y<<std::endl;
  // optimize(calib_datas);
  return 0;

}


