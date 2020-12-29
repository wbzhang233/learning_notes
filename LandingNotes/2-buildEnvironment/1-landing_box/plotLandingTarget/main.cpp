#include <iostream>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

#define ROWS 1800
#define COLS 1200
#define RADIUS 125
#define INTERVAL 187.5
#define oriX 431.25+5*INTERVAL
#define oriY COLS/2

void trans(Point2f &point){
    swap(point.x,point.y);
}

int main() {
    std::cout << "Hello, World!" << std::endl;
    cv::Mat plot=cv::Mat(ROWS,COLS,CV_8UC3,cv::Scalar(0,0,255));

    Point2f origin(oriX,oriY);// 中间点
    Point2f point1(oriX-5*INTERVAL,oriY); // 顶点,绿色
    Point2f point2(oriX-3*INTERVAL,oriY+INTERVAL); // 右上
    Point2f point3(oriX,oriY+2*INTERVAL); // 右边
    Point2f point5(oriX,oriY-2*INTERVAL);// 左边
    Point2f point6(oriX-3*INTERVAL,oriY-INTERVAL);// 左上
    trans(origin);trans(point1);trans(point2);
    trans(point3);trans(point5);trans(point6);

    // 绿色点
    cv::circle(plot,point1,RADIUS,CvScalar(0,255,0),-1,8);
    // 蓝色点
    cv::circle(plot,point2,RADIUS,CvScalar(255,0,0),-1,8);
    cv::circle(plot,point3,RADIUS,CvScalar(255,0,0),-1,8);
    cv::circle(plot,origin,RADIUS,CvScalar(255,0,0),-1,8);
    cv::circle(plot,point5,RADIUS,CvScalar(255,0,0),-1,8);
    cv::circle(plot,point6,RADIUS,CvScalar(255,0,0),-1,8);

    cv::imshow("landingTarget",plot);
    cv::waitKey();
    cv::imwrite("landingTarget.png",plot);
    std::cout << "Done!" << std::endl;
    return 0;
}