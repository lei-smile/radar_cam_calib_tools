#include "cam2RTK_calib.h"
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
using namespace GeographicLib;

bool ReadConfigfile(std::string config_fileName)
{
  cv::FileStorage config_read(config_fileName, cv::FileStorage::READ);
  if(!config_read.isOpened())
  {
    std::cout<<"Read the calib config file failed！"<<std::endl;
    return false;
  }
  config_read["calib_path"] >> calib_path;
  config_read["cam_file"] >> cam_file;
  cam_file = calib_path + cam_file;
  config_read["image_file"] >> image_file;
  image_file = calib_path + image_file;
  config_read["result_file"] >> result_file;
  result_file = calib_path + result_file;
  config_read["cam_reproject_file"] >> cam_reproject_file;
  cam_reproject_file = calib_path + cam_reproject_file;
  config_read["lat0"] >> lat0;
  config_read["lon0"] >> lon0;
  config_read["height0"] >> height0;

  config_read.release();
  return true;
}

bool ReadCalibfile(std::string cam_fileName, std::vector<CalibData>& points_calib)
{
  points_calib.clear();

  static const Geocentric& earth = Geocentric::WGS84();
  LocalCartesian gps_trans(lat0, lon0, height0, earth);

  std::ifstream cam_ifile(cam_fileName);
  if (!cam_ifile.is_open())
  {
    std::cout<<"Read the cam calib data failed！"<<std::endl;
    return false;
  }
  std::string cam_lineStr;
  while (std::getline(cam_ifile, cam_lineStr))
  {
    CalibData dataInfo;
    std::stringstream cam_ss(cam_lineStr);
    std::string cam_str;
    double lat, lon, height;
    std::getline(cam_ss, cam_str, ','); dataInfo.u = atof(cam_str.c_str());
    std::getline(cam_ss, cam_str, ','); dataInfo.v = atof(cam_str.c_str());
    std::getline(cam_ss, cam_str, ','); lat = atof(cam_str.c_str());
    std::getline(cam_ss, cam_str, ','); lon = atof(cam_str.c_str());
    std::getline(cam_ss, cam_str, ','); height = atof(cam_str.c_str());

    gps_trans.Forward(lat, lon, height, dataInfo.x, dataInfo.y, dataInfo.z);
    std::cout<<dataInfo.u<<" "<<dataInfo.v<<" "<<dataInfo.x
             <<" "<<dataInfo.y<<" "<<dataInfo.z<<std::endl;
    points_calib.push_back(dataInfo);
  }
  cam_ifile.close();
  return true;
}

std::vector<cv::Mat> GetUtm2Cam()
{
  std::vector<cv::Point2f> objPoints;
  std::vector<cv::Point2f> imgPoints;
  std::vector<cv::Mat> umt_cam;
  imgPoints.clear();
  objPoints.clear();
  // int Id_a = int(candidate_matrix.at<double>(0,0))-1;
  // int Id_b = int(candidate_matrix.at<double>(1,0))-1;
  // int Id_c = int(candidate_matrix.at<double>(2,0))-1;
  // int Id_d = int(candidate_matrix.at<double>(3,0))-1;

  // imgPoints.push_back(cv::Point2f(points_calib[Id_a].u,points_calib[Id_a].v));
  // imgPoints.push_back(cv::Point2f(points_calib[Id_b].u,points_calib[Id_b].v));
  // imgPoints.push_back(cv::Point2f(points_calib[Id_c].u,points_calib[Id_c].v));
  // imgPoints.push_back(cv::Point2f(points_calib[Id_d].u,points_calib[Id_d].v));
  // objPoints.push_back(cv::Point2f(points_calib[Id_a].x,points_calib[Id_a].y));
  // objPoints.push_back(cv::Point2f(points_calib[Id_b].x,points_calib[Id_b].y));
  // objPoints.push_back(cv::Point2f(points_calib[Id_c].x,points_calib[Id_c].y));
  // objPoints.push_back(cv::Point2f(points_calib[Id_d].x,points_calib[Id_d].y));

  for(int i = 0; i < points_calib.size(); i++ )
  {
    imgPoints.emplace_back(cv::Point2f(points_calib[i].u, points_calib[i].v));
    objPoints.emplace_back(cv::Point2f(points_calib[i].x, points_calib[i].y));
  }

  // cv::Mat cam2UTM = cv::getPerspectiveTransform(imgPoints, objPoints);
  cv::Mat cam2UTM = cv::findHomography(imgPoints, objPoints);
  umt_cam.push_back(cam2UTM);
  // cv::Mat UTM2cam = cv::getPerspectiveTransform(objPoints, imgPoints);
  cv::Mat UTM2cam = cv::findHomography(objPoints, imgPoints);
  umt_cam.push_back(UTM2cam);

  std::cout<<"cam2UTM"<<cam2UTM<<std::endl;
  std::cout<<"UTM2cam"<<UTM2cam<<std::endl;

  return umt_cam;
}

cv::Mat DrawUtm2Cam(std::vector<cv::Point2d> objectPoints,
                    std::vector<cv::Point2d> imagePoints)
{
   cv::FileStorage result_write(result_file, cv::FileStorage::WRITE);
   std::vector<cv::Mat> umt_cam = GetUtm2Cam();
   cv::Mat image  = cv::imread(image_file);

   std::vector<cv::Point2f> utmPos, pixelPos;
   for (int i = 0; i < objectPoints.size(); i++)
   {
     utmPos.push_back(cv::Point2f(objectPoints[i].x, objectPoints[i].y));
   }
    
   cv::perspectiveTransform(utmPos, pixelPos, umt_cam[1]);

   double sum_error = 0;

   for (int i = 0; i < objectPoints.size(); ++i)
   {
      cv:Point2d p1,p2;

      p1.x = imagePoints[i].x;
      p1.y = imagePoints[i].y;
      circle(image, p1, 10, Scalar(0, 255, 0),4);
      //std::cout<<"************src:"+to_string(i) +"   ("<<p.x<<","<<p.y<<")"<<std::endl;

      p2.x = pixelPos[i].x;
      p2.y = pixelPos[i].y;
      circle(image, p2, 10, Scalar(0, 0, 255),4);
      //std::cout<<"************obj:"+to_string(i) +"   ("<<p.x<<","<<p.y<<")"<<std::endl;
      sum_error += std::hypot(std::abs(p1.x-p2.x),std::abs(p1.y-p2.y));
   }

   result_write << "cam2UTM" << umt_cam[0] << "UTM2cam" << umt_cam[1];
   result_write << "reprojection_error" << sum_error/objectPoints.size();
   //result_write.release();
   std::cout<<"sum_error:  "<<sum_error<<std::endl;
   std::cout<<"reprojection_error: "<<sum_error/objectPoints.size()<<std::endl;
   return image;
}

int main(int argc, char *argv[])
{
  std::string config_file = "../config/calib_config.yaml";
  ReadConfigfile(config_file);
  ReadCalibfile(cam_file, points_calib);

  std::vector<cv::Point2d > objectPoints;
  std::vector<cv::Point2d > imagePoints;
  for (int i = 0; i < points_calib.size(); i++ )
  {
    cv::Point2d im_point;
    cv::Point3d world_point;
    imagePoints.push_back(cv::Point2d(points_calib[i].u, points_calib[i].v));
    objectPoints.push_back(cv::Point2d(points_calib[i].x, points_calib[i].y));
  }
  cv::Mat image = DrawUtm2Cam(objectPoints, imagePoints);
  cv::Mat image2;
  cv::resize(image,image2,cv::Size(image.cols*0.4,image.rows*0.4),0,0,0);
  cv::imshow("image",image2);
  cv::imwrite(cam_reproject_file, image2);
  //IplImage image_saved;
  //image_saved = IplImage(image2);
  //cvSaveImage("../dataset/01_origin.jpg", &image_saved);
  cv::waitKey();

  return 0;
}

// int main()
// {
//   double lat0 = 31.373718, lon0 = 120.4203869, height0 = 11.737;
//   static const Geocentric& earth = Geocentric::WGS84();
//   LocalCartesian gps_trans(lat0, lon0, height0, earth);
//   double lat = 31.37447572, lon = 120.4202786, height = 11.916;
//   double x = 0, y = 0, z= 0;
//   gps_trans.Forward(lat, lon, height, x, y, z);
//   std::cout<<"x:  "<<x<<"y:  "<<y<<"z:   "<<z<<std::endl;


// }






