#include<opencv2/opencv.hpp>
#include<fstream>
using namespace std;

std::vector<cv::Point> calib_points;
std::string image_file, calib_points_file;
std::string calib_path;
cv::Mat image_down;
int num_point = 0;

bool ReadConfigfile(std::string config_fileName)
{
  cv::FileStorage config_read(config_fileName, cv::FileStorage::READ);
  if(!config_read.isOpened())
  {
    std::cout<<"Read the calib config file failed！"<<std::endl;
    return false;
  }

  config_read["calib_path"] >> calib_path;
  config_read["image_file"] >> image_file;
  image_file = calib_path + image_file;
  std::cout<<"img_file: "<<image_file<<std::endl;
  config_read["output_file"] >> calib_points_file;
  calib_points_file = calib_path + calib_points_file;

  return true;
}

void onMouse(int event, int x, int y, int flags, void *param)
{
  cv::Mat *im = reinterpret_cast<cv::Mat*>(param);
  
  
  switch (event){
  case CV_EVENT_LBUTTONDOWN:
  {
    num_point++;
    cout << "point"<<num_point<<": (" << x/0.5 << "," << y/0.5 << ")   value is:" << static_cast<int>
      (im->at<uchar>(cv::Point(x, y))) << endl;
    cv::Point calib_point = cv::Point(x/0.5, y/0.5);
    calib_points.push_back(calib_point);
    cv::circle(image_down, cv::Point2f(x, y),3, cv::Scalar(0, 255, 0), -1);
    cv::imshow("image", image_down);
    cv::waitKey(1);
  }
    break;
  }
}

void SavedCalibPoints()
{
  ofstream points_save(calib_points_file,std::fstream::out);
  for(int i = 0; i< calib_points.size(); i++)
  {
    points_save << calib_points[i].x << "," << calib_points[i].y << std::endl;
  }
}
  
int main()
{
  std::string config_file = "../config/getPixel.yaml";
  ReadConfigfile(config_file);
  
  cv::Mat image = cv::imread(image_file);
  if(image.rows != 1080 || image.cols != 1920)
  {
    std::cout<<"image resolution is not correct!!"<<std::endl;
    return 0;
  }
  cv::namedWindow("image", CV_WINDOW_NORMAL);
  cv::resize(image,image_down,cv::Size(image.cols*0.5,image.rows*0.5),0,0,0);
  cv::setMouseCallback("image", onMouse, reinterpret_cast<void *>(&image_down));
  cv::imshow("image", image_down);
  cv::waitKey(0);
  SavedCalibPoints();
  return 0;
}







