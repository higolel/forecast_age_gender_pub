#ifndef __BASE64_MAT_H__
#define __BASE64_MAT_H__
#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <face_plate_msgs/Face_pic.h>
#include <thread>
#include <queue>
#include <vector>
#include <opencv2/freetype.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ctime>
#include <sys/time.h>
#include <algorithm>
#include <string>
#include <stdlib.h>
#include <dynamic_reconfigure/server.h>
#include <sys/select.h>

std::string base64Decode(const char *Data, int DataByte);
std::string base64Encode(const unsigned char *Data, int DataByte);
std::string Mat_to_Base64(cv::Mat img, std::string imgType);
cv::Mat Base_to_Mat(std::string &base64_data);

#endif
