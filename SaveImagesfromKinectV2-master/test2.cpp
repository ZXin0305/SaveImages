#include<iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include<string>

using namespace std;
using namespace cv;

int main(int argc, char** argv){
    // string file_path = "/media/xuchengjun/disk/datasets/SaveImagesfromKinectV2-master/build/062422/062422_videos/depth_frame/121.png";
    // cv::Mat depth = cv::imread(file_path);
    // cv::Vec3f depth_channel = depth.at<Vec3f>(888, 395);
    // cout<<depth_channel.val[0]<<" "<<depth_channel[1]<<" "<<depth_channel[2]<<endl;
    // // cout<<depth.at<uint>(960, 540)<<endl;
    // cv::imshow("img", depth);
    // cv::waitKey(0);

    FileStorage ffs("/media/xuchengjun/disk/datasets/SaveImagesfromKinectV2-master/build/Pos.xml", FileStorage::READ);
    cv::Mat vocabulary123;
    ffs["opencv_storage"]["depth"]["data"] >> vocabulary123;
    ffs.release();
    // cv::imshow("img", vocabulary123);
    // cv::waitKey(0);
    return 0;
}