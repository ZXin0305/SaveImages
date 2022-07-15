#include <iostream>
#include <stdio.h>
#include <iomanip>
#include <time.h>
#include <signal.h>
#include <opencv2/opencv.hpp>
#include<vector>

#include <chrono>
#include <sys/stat.h>
#include <dirent.h>
#include <errno.h>

using namespace std;
using namespace cv;

int main(int argc, const char** argv)
{
  std::string folder_input;
  std::string save_folder_name;
  string folder_rgb;
  string folder_depth;
  // std::string base_directory;

  std::cout << "Enter the Person name you want to read bin files of: ";
  std::cin >> folder_input;
  const char * Image_base_name = folder_input.c_str();

  std::cout << "Enter the generic name of folder you want to save your images/videos with: ";
  std::cin >> save_folder_name;   //这个是用于后面提取.bin文件中的depth image, 一个中间变量
  const char * saveimagebasename = save_folder_name.c_str();

  // // 20220714 直接保存成 png
  // cout<< "Enter RGB image store path: ";
  // cin >> folder_rgb;
  // const char * saveRGBbasename = folder_rgb.c_str();

  // cout<<"Enter depth image store path: ";
  // cin >> folder_depth;
  // const char * saveDeptbasename = folder_depth.c_str();
  // // 20220714

  // 在这里的base_directory必须设置成./bin文件所在的地方
  cv::String base_directory = "./";                                         //根路径
  cv::String mainfolder = base_directory + Image_base_name;                 //保存路径
  cv::String Image_name = mainfolder + "/" + Image_base_name + "_%d.bin";   //读取的路径
  // <<< String Initialization for Reading >>> Single .bin file

  // // >>> String Initialization for Writing
  cv::String base_directory_write = base_directory;
  cv::String mainfolder_write = base_directory_write + Image_base_name;  // --> ./1

  cv::String depth_foldername = mainfolder_write + "/depth_images";
  cv::String depth_Image_name_png = depth_foldername + "/" + saveimagebasename + "_d_%d.png";   // e.g. ./1/depth_images/11_d_195.png
  char filename_depth[200];  // 中间生成的depth map的名称

  // 20220714  两个文件夹
  cv::String rgb_img_folder = mainfolder_write + "/" + folder_input + "_rgb_frame";
  cv::String depth_img_folder = mainfolder_write + "/" + folder_input + "_depth_frame";
  // 20220714

  cv::String videofolder = mainfolder_write + "/" + folder_input + "_videos";
  cv::String rgbvideoname = videofolder + "/rgb_video.avi";  // RGB视频
  cv::String depthvideoname = videofolder + "/depth_video.avi";  // depth map拼接成的视频

  DIR* dir = opendir(videofolder.c_str());
  if(!dir)
  {
    const int dir_err_video = mkdir(videofolder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if(-1 == dir_err_video)
      {
        std::cout << "Error creating video folder!" << std::endl;
        exit(1);
      }
  }
  else closedir(dir);

  DIR* dir1 = opendir(rgb_img_folder.c_str());
  if(!dir1)
  {
    const int dir1_err_video = mkdir(rgb_img_folder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if(-1 == dir1_err_video)
      {
        std::cout << "Error creating video folder!" << std::endl;
        exit(1);
      }
  }
  else closedir(dir1);

  DIR* dir2 = opendir(depth_img_folder.c_str());
  if(!dir2)
  {
    const int dir2_err_video = mkdir(depth_img_folder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if(-1 == dir2_err_video)
      {
        std::cout << "Error creating video folder!" << std::endl;
        exit(1);
      }
  }
  else closedir(dir2);

  // 文件夹创建成功

  char filename[80];  //.bin文件的名称
  FILE * pFile;
  uchar * rgbbuffer;
  float * depthbuffer;
  int * sizeofimage;

  size_t result;
  long totalbytesread;
  long bytesofsizeofdepthimage;

  int initialframe = 1;   //可以进行设置,对文件进行挑选
  std::string input;

  std::vector<cv::String> filenames;

  cv::glob(mainfolder, filenames);  //得到所有的文件的数量
  std::cout << "Total number of frames in this folder: " << filenames.size() << std::endl;
  std::cout << "Start from frame number [default=1]: ";
  std::cin >> input;
  // std::getline( std::cin, input );
  if ( !input.empty() ) {
      std::istringstream stream( input );
      stream >> initialframe;
  }
  std::cout << "Value of Initial Frame selected: " << initialframe << std::endl;

  cv::VideoWriter rgbVideo, depthVideo;

  int fourcc = CV_FOURCC('D','I','V','X');
  int fps = 18;

  rgbVideo.open(rgbvideoname, fourcc, fps, cv::Size(1920, 1080), true);
  if (!rgbVideo.isOpened())
    {
      std::cout  << "Could not open the rgb video for write: " << std::endl;
      return -1;
    }
  depthVideo.open(depthvideoname, fourcc, fps, cv::Size(1920, 1082), true);
  if (!depthVideo.isOpened())
    {
      std::cout  << "Could not open the depth video for write: " << std::endl;
      return -1;
    }

  cv::Mat intdepth, depth_int_onechannel, depth_floatto255;
  cv::Mat depth_multichannel;

  cv::namedWindow("RGB Image", cv::WINDOW_NORMAL);
  cv::namedWindow("Depth Image", cv::WINDOW_NORMAL);


  // FileStorage fs("Pos.xml", FileStorage::WRITE);
  for(int i = initialframe; i < filenames.size(); i++)
  // for(int i = initialframe; i < 100; i++)
    {
      // // >>> filenames for writing
      sprintf(filename_depth,depth_Image_name_png.c_str(),i);  // sprintf --> 把格式化的数据写入某个字符串缓冲区。
      // // <<< filenames for writing

      // >>> Only .bin File with RGB and Depth reading
      sprintf(filename, Image_name.c_str(), i);
      // <<< Only .bin File with RGB and Depth reading

      std::cout << "Processing file: " << filename << std::endl;
      pFile = fopen ( filename , "rb" );  // 打开.bin文件

      fseek (pFile , 0 , SEEK_END);   // fseek --> 重定位流(数据流/文件)上的文件内部位置指针
      totalbytesread = ftell (pFile);   // 获取文件的 当前指针位置 相对于 文件首地址 的 偏移字节数 --> 即：获取当前文件的总的字节大小

      bytesofsizeofdepthimage = sizeof(int)*2;
      rewind (pFile);

      sizeofimage = (int*) malloc (2);

      result = fread(sizeofimage, 1, bytesofsizeofdepthimage, pFile);  // C 库函数 size_t fread(void *ptr, size_t size, size_t nmemb, FILE *stream) 
                                                                       // 从给定流 stream 读取数据到 ptr 所指向的数组中 --> 读取二进制文件的方法
                                                                       // ptr: 是读取的数据存放的内存的指针（可以是数组，也可以是新开辟的空间,就是一个索引）
                                                                       // size: 每次读取的字节数 
                                                                       // nmemb: 读出size字节的数据项的个数
                                                                       // stream: 目标文件指针
      if (result != bytesofsizeofdepthimage){
        fputs ("Reading error",stderr); exit (3);
      }
      long bytesofdepthimage = sizeof(float)*sizeofimage[0]*sizeofimage[1];   //一个bin文件的depth map的存储大小 
      long bytesofrgbimage = totalbytesread - bytesofsizeofdepthimage - bytesofdepthimage; //得到一个bin文件中RGB图像的字节大小  减去bytesofdepthimage是因为保存的时候保存了这个

      rgbbuffer = (uchar*) malloc (bytesofrgbimage);  //提前申请好空间
      depthbuffer = (float *) malloc (bytesofdepthimage); // malloc assigns memory block in bytes

      // 读取RGB图像
      result = fread(rgbbuffer, 1, bytesofrgbimage, pFile);
      if (result != bytesofrgbimage){
        fputs ("Reading error",stderr); exit (3);
      }

      // 读取depth map
      result = fread(depthbuffer, 1, bytesofdepthimage, pFile);
      if (result != bytesofdepthimage){
        fputs ("Reading error",stderr); exit (3);
      }
      fclose(pFile);  //关闭bin文件

      //读取RGB文件
      cv::Mat readdata = cv::Mat(1,bytesofrgbimage, CV_8UC1, rgbbuffer);
      cv::Mat src = readdata.reshape(4, 1080);
      cv::flip(src, src, 1);  //将成像翻转

      //读取depth map
      cv::Mat readdepthdata = cv::Mat(1,bytesofdepthimage / sizeof(float), CV_32FC1, depthbuffer);
      cv::Mat src_d = readdepthdata.reshape(0, sizeofimage[0]);
      cv::flip(src_d, src_d, 1);

      // For saving rgb images and videos we need to convert BGRA to BGR
      cv::cvtColor(src, src, cv::COLOR_BGRA2BGR);  // kinect得到的图像的编码格式是BGRA8格式的，因此在后面制作数据集的时候需要在video wrapper处进行转换  --> 其实不用，cv2好像会自动转化到RGB进行显示
                                                   // 

      // For display only we need to convert the float depth grayscale into
      // CV_8U 3 channels. We will replicate the single channel data into
      // three channels and will merge the array of matrices to form an image.
      depth_floatto255 = src_d / 4096.0f * 255;  // 对距离进行一个归一化,再将其映射到[0,255]之间

      // std::cout<< depth_floatto255.at<float>(540, 960)<<std::endl;
      // double depth_sum = 0.0;
      // for(int i = 540 - 25; i < 540 + 25; ++i){
      //   for(int j = 960 - 25; j < 960 + 25; ++j){
      //       depth_sum += depth_floatto255.at<float>(i, j) / 255;
      //   }
      // }
      // std::cout<<depth_sum / 2500 * 4096<<std::endl;  //这样得到的深度值以mm为单位

      depth_floatto255.convertTo(depth_int_onechannel, CV_8UC1);   //转换成RGB的数据格式  在这里转换成了CV_8UC1(这个是RGB图的数据类型)
      // 叠加成三通道
      // cv::Mat depth_temp[] = {depth_int_onechannel, depth_int_onechannel, depth_int_onechannel};
      vector<cv::Mat> channels(3, depth_int_onechannel);
      // if (i == 120 || i == 121){
      //     //  fs << "depth" << depth_int_onechannel;
      //     cv::imwrite("depth.png", depth_int_onechannel);  // 保存成png可以直接变成相同的三通道诶
      //   float depth_sum = 0.0;
      //   for(int c = 540 - 25; c<540 + 25; ++c){
      //     for(int r = 960 - 25; r < 960 + 25; ++r){
      //       int val = int(channels[1].at<uint8_t>(c, r));
      //       depth_sum += val / 255.0 * 4096.0f;
      //     }
      //   }
      //   cout<<depth_sum / 2500.0 <<endl;
      // }
      cv::merge(channels, depth_multichannel);
      cv::imshow("Depth Image", depth_multichannel);

      // 视频保存
      // >>> Transferring the rgb (BGR) and depth images to video writer
      // rgbVideo << src;
      // depthVideo << depth_multichannel;
      // <<< Transferring the rgb (BGR) and depth images to video writer

      // std::cout << filename_depth << std::endl;

      // >>> Writing Color and depth images (both in png)
      // cv::imwrite(filename_depth, depth_multichannel);
      cv::imwrite(rgb_img_folder + "/rgb_" + to_string(i) + ".png", src);
      cv::imwrite(depth_img_folder + "/depth_" + to_string(i) + ".png", depth_int_onechannel);
      // >>> Writing Color and depth images (both in png)

      cv::imshow("RGB Image", src);
      cv::imshow("Depth Image", depth_multichannel);
      cv::waitKey(1);

      free (rgbbuffer);
      free (depthbuffer);
      free (sizeofimage);

      // free (buffer);
      // free (sizeofimage);
    }
    rgbVideo.release();
    depthVideo.release();
    // fs.release();

}
