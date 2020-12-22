#ifndef FORECAST_AGE_GENDER_H
#define FORECAST_AGE_GENDER_H

#include "base64_mat.h"

using namespace std;

enum ImageFormat {
    RGBA = 0,
    RGB,
    BGR,
    GRAY,
    BGRA,
    YUV_NV21 = 11,
    YUV_NV12 = 12,
};

enum FaceSDKMode{
    Normal,
    Android_Front_Camera,
    Android_Back_Camera,
};

enum ModelType{
    Detect,
    Landmark,
    Recognize,
    Attribution,
    Landmark3d,
    GoDetect,
    Iris,
    BlazePose,
    BlazePoseLandmark,
    HandDetect,
    HandLandmark3d,
};

typedef struct FaceSDKConfig
{
    int img_w;
    int img_h;
    int screen_w;
    int screen_h;

    ImageFormat input_format;
    FaceSDKMode mode;
    int thread_num;
} FaceSDKConfig;

typedef struct FaceAttribution{
    int gender; // 0 man, 1 woman
    int glasses;
    int age;
    int smile;
    int beauty_man_look;
    int beauty_woman_look;
} FaceAttribution;

typedef struct Box {
    float x1;
    float y1;
    float x2;
    float y2;
}Box;

typedef struct FaceInfo {
    Box face_box;
    float score;
    float head_x;
    float head_y;
    float head_z;
    float lefteye_close_state;
    float righteye_close_state;
    float mouth_close_state;
    float mouth_bigopen_state;
    float landmarks[212 * 2];
    float person_mark[512];
    FaceAttribution attribution;
} FaceInfo;

typedef struct sdkFaces {
    int face_count = 0;
    FaceInfo* info;
} sdkFaces;


std::string model_detect_, model_attribution_, ch_ttf_;
image_transport::Publisher pub_image_;

ros::Publisher pub_face_pic_message_;
ros::Subscriber sub_pic_;

queue<cv::Mat> que_cv;

void facesdk_init(FaceSDKConfig config);
void facesdk_readModelFromFile(ModelType type, const char* model_path, ImageFormat modelInputFormat);
sdkFaces facesdk_detect(char *imgData);
sdkFaces facesdk_attribute();

void sleep_ms(unsigned int millisec);
void Add_text_to_pic(cv::Mat &image, std::string text, cv::Point origin);

void FacePicMsgCallback(const face_plate_msgs::Face_pic::ConstPtr &msg);
void Face_struct_assignment(face_plate_msgs::Face_pic &face_msg, const face_plate_msgs::Face_pic::ConstPtr msg);
#endif
