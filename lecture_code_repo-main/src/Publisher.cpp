#include <bits/stdc++.h>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge.hpp>
using namespace std;
using namespace cv;
cv::Mat res;
std::pair<double ,double> pai; 
Point2f auto_aim(int color,Mat frame){
    Point2f midPoint;
    Scalar lower_color, upper_color; // 颜色阈值（红/蓝）
    string video_path;               // 视频路径（红/蓝）
    if (color == 0) {
        lower_color = Scalar(0, 50, 235);
        upper_color = Scalar(45, 255, 255);
        // video_path = "../videos/red.mp4";//红色颜色
    } else if (color == 1) {
        lower_color = Scalar(85, 50, 245);
        upper_color = Scalar(135, 255, 255);
        // video_path = "../videos/blue.mp4";//蓝色颜色
    } 
    // VideoCapture cap(video_path);


    
        // namedWindow("装甲板识别结果", WINDOW_NORMAL);

    // cap >> frame;
    Mat blur_frame, hsv_frame, mask;
    GaussianBlur(frame, blur_frame, Size(5, 5), 10, 20);//转HSV格式
    cvtColor(blur_frame, hsv_frame, COLOR_BGR2HSV);//高斯模糊，去噪声
    inRange(hsv_frame, lower_color, upper_color, mask);//形态学闭运算
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(mask, mask, MORPH_CLOSE, kernel, Point(-1, -1), 30);

//检测轮廓
    vector<vector<Point>> contours;    // 轮廓
    vector<Vec4i> hierarchy;           // 层级关系
    vector<Point2f> centers;           // 矩形几何中心
    findContours(mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point());
      
//绘制灯条,找中心
    
    for (int n = 0; n < contours.size(); n++) {
            RotatedRect rrect = minAreaRect(contours[n]);
            Point2f points[4];
            rrect.points(points);
            Point2f cpt = rrect.center;
            centers.push_back(cpt);
            circle(frame, cpt, 2, Scalar(255, 0, 0), 2, 8, 0);
        }
        
//进行连线
        for (size_t i = 0; i < centers.size(); i++) {
            if (i < centers.size() - 1) {
                Point2f start = centers[i];
                Point2f end = centers[i+1];                           // 绘制灯条连接线（红色细线）
                // line(frame, start, end, Scalar(0, 0, 255), 3, 8, 0); // 计算并绘制装甲板中心（绿色实心大圆）
                midPoint.x=(start.x + end.x) / 2;
                midPoint.y=(start.y + end.y) / 2;
                // circle(frame, midPoint, 15, Scalar(0, 255, 0), -1, 8, 0);
            }
        }
    
    return midPoint;
}

class Rece : public rclcpp::Node
{
  private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub;
  public:
  void recall(sensor_msgs::msg::Image::SharedPtr A)
  {
    res=cv_bridge::toCvShare(A,"bgr8")->image;
    Point2f pray;

    pray=auto_aim(0,res);
    cout<<pray.x<<" "<<pray.y;
  }

  Rece(std::string name):Node(name)
  {
    this->create_subscription<sensor_msgs::msg::Image>("/image_raw",10,std::bind(&Rece::recall,this));
  }


};
int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  auto kkk=make_shared<Rece>("subsc");
  rclcpp::spin(kkk);
  rclcpp::shutdown();
  return 0;
}
