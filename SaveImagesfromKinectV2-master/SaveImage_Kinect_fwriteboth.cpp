//! [headers]
#include <iostream>
#include <stdio.h>
#include <iomanip>
#include <time.h>
#include <signal.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

// kinect的头文件
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
//! [headers]
#include <chrono>
#include <sstream>
#include <fstream>
#include <sys/stat.h>
/* 文件夹操作函数  */
#include <dirent.h>
/* unix/linux系统定义文件状态所在的伪标准头文件 */
#include <string>
#include <errno.h>
#include<ctime>

using namespace std;
using namespace cv;

bool protonect_shutdown = false; // Whether the running application should shut down.

void sigint_handler(int s)
{
  protonect_shutdown = true;
}

const char* keys =
{
  "{savedepth |1| whether to save depth image or not, 1 yes, 0 no}"
};

int main(int argc, const char** argv)
{
    clock_t st, et;
    //CommandLineParser类：命令行解析类
    //方便用户在命令行使用过程中减少工作量，可以在程序文件中直接指定命令行中的参数指令，方便调试
    cv::CommandLineParser parser(argc, argv, keys);

    int depthsave = parser.get<int>("savedepth");   //是否保存深度图

    std::string folder_rgb, name_rgb;
    std::string folder_d, name_d;
    std::string gesture_label, subject_idx;

    //RGB图像保存路径
    std::cout << "Enter the name of folder you want to save your images in: ";
    std::cin >> folder_rgb;

    const char * folder_name = folder_rgb.c_str(); //转换成c风格的char型指针

    // cout<< folder_name <<endl;

    DIR* dir = opendir(folder_name);  //DIR表示目录类型，  若成功则返回指针，若出错则返回NULL

    //上面的成功返回了指针，说明该文件夹存在,函数中关闭这个文件夹，并重新输入，直到输入一个新的文件夹名称
    while(dir)
    {
      closedir(dir);
      std::cout << "Folder already exists; re-enter the name of the folder: ";
      std::cin >> folder_rgb;
      dir = opendir(folder_name);
    }

    if (ENOENT == errno)
    {
      const int dir_err = mkdir(folder_name, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
      if(-1 == dir_err)
        {
          std::cout << "Error creating folder!" << std::endl;
          exit(1);
        }
    }
    else
      {
        std::cout << "Cannot check if folder exists!" << std::endl;
        std::cout << "Closing Program!" << std::endl;
        exit(1);
      }

    std::cout << "Streaming from Kinect One sensor!" << std::endl;

    // Initialize and Discover Devices
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;    //传感器设备, 0就是个地址
    libfreenect2::PacketPipeline *pipeline = 0; //数据传输方式

    //! [discovery]
    // 检测设备，返回的是一个列表
    if(freenect2.enumerateDevices() == 0)
    {
        std::cout << "no device connected!" << std::endl;
        return -1;
    }

    std::string serial = freenect2.getDefaultDeviceSerialNumber();

    std::cout << "SERIAL: " << serial << std::endl;   // 003415165047 -- kinect设备的序列号

    if(pipeline)
    {
        //! [open]
        // Open and Configure the Device
        dev = freenect2.openDevice(serial, pipeline);   //按照某种数据传输方式打开设备
        //! [open]
    } else {
        dev = freenect2.openDevice(serial);
        // cout<<"新的设备号码"<<dev<<endl;  //检测得到的设备重新赋予一个设备地址
    }
    // 再次检测设备是否不存在
    if(dev == 0)
    {
        std::cout << "failure opening device!" << std::endl;
        return -1;
    }

    signal(SIGINT, sigint_handler);
    protonect_shutdown = false;

    //! [listeners]
    // 同步多窗口监听器, 监听对象包括 RGB/depth
    // You cannot configure the device after starting it.
    libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color |
                                                  libfreenect2::Frame::Depth |
                                                  libfreenect2::Frame::Ir);
    libfreenect2::FrameMap frames;

    dev->setColorFrameListener(&listener);   //RGB
    dev->setIrAndDepthFrameListener(&listener); //Depth

    //! Start the Device
    // 开始监听
    dev->start();

    std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
    //! [start]

    //! [registration setup]
    libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4), depth2rgb(1920, 1080 + 2, 4); // check here (https://github.com/OpenKinect/libfreenect2/issues/337) and here (https://github.com/OpenKinect/libfreenect2/issues/464) why depth2rgb image should be bigger
    //! [registration setup]

    cv::Mat rgbmat, depthmatUndistorted, irmat, rgbd, depth_fullscale;
    unsigned int i = 1;

    // Receive Image Frames
    int count = 0;
    st = clock();
    while(!protonect_shutdown)
    {
        if (!listener.waitForNewFrame(frames, 10*1000)) // 10 sconds
        {
          std::cout << "timeout!" << std::endl;
          return -1;
        }

        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
        //! [loop start]

        auto tinit = std::chrono::high_resolution_clock::now();

        cv::Mat rgbmat(rgb->height, rgb->width, CV_8UC4, rgb->data);  //将从libfreenect2类订阅得到的RGB图像转换到opencv
        cv::Mat depthmat(depth->height, depth->width, CV_32FC1, depth->data); //同样，深度图

        //! [registration]
        registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);
        //! [registration]

        cv::Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data).copyTo(depthmatUndistorted);
        cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(rgbd);
        cv::Mat(depth2rgb.height, depth2rgb.width, CV_32FC1, depth2rgb.data).copyTo(depth_fullscale);  //转换到和RGB图像一个scale,单通道
        
        count++;
        et = clock();
        // cout<<et - st<<endl;
        if((et - st) / CLOCKS_PER_SEC == 1){
          cout<<"***************\t" << count << "\t******************************"<<endl;
          count = 0;
          st = et;
        }
        // 显示
	      cv::namedWindow("RGB Image", cv::WINDOW_NORMAL);
        cv::imshow("RGB Image", rgbmat);
        cv::namedWindow("Depth Map", cv::WINDOW_NORMAL);
        cv::imshow("Depth Map", depth_fullscale / 4096.0f);   //除以4096.0f是因为kinect的官方最大有效距离是4096mm,这里是对距离进行了归一化
                                                              

        // double depth_sum = 0.0;
        // for(int i = 540 - 25; i < 540 + 25; ++i){
        //   for(int j = 960 - 25; j < 960 + 25; ++j){
        //       depth_sum += depth_fullscale.at<float>(i, j);
        //   }
        // }
        // cout<<depth_sum / 2500 <<endl;  //这样得到的深度值以mm为单位
        
        // IMPORTANT TO PUT THIS PATH CORRECT //
        std::stringstream imagename;
        imagename << "./" << folder_rgb << "/" << folder_rgb << "_" << i << ".bin";

        int sizeofdepthimage[2] = {depth_fullscale.rows, depth_fullscale.cols};

        // 保存视频
        std::fstream file(imagename.str(), std::ios::binary | std::ios::out);
        file.write((char*)sizeofdepthimage, 2*sizeof(int));
        file.write((char*)rgbmat.data, rgbmat.channels() * rgbmat.rows * rgbmat.cols*sizeof(uchar));   //默认RGB和depth都进行保存
        file.write((char*)depth_fullscale.data, depth_fullscale.rows * depth_fullscale.cols*sizeof(float));

        auto twriteimage = std::chrono::high_resolution_clock::now();

        std::chrono::duration<double, std::milli> fp_ms = twriteimage-tinit;
        std::cout << "duration write depth: "<< fp_ms.count() << "\t frame number:" << i << std::endl;  //毫秒
        std::cout << "--------------------------------------------------------"<< std::endl;

        i++;

        int key = cv::waitKey(1);
        protonect_shutdown = protonect_shutdown || (key > 0 && ((key & 0xFF) == 27)); // shutdown on escape

        //! [loop end]
        listener.release(frames);
        // libfreenect2::this_thread::sleep_for(libfreenect2::chrono::milliseconds(100));
    }

    //! [stop]
    dev->stop();
    dev->close();
    //! [stop]

    delete registration;

    std::cout << "Streaming Ends!" << std::endl;
    return 0;
}
