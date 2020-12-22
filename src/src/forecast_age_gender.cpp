#include "forecast_age_gender.h"

// --------------------------------------------------------
/// \概要:	msleep
///
/// \参数:	image
// --------------------------------------------------------
void sleep_ms(unsigned int millisec)
{
	struct timeval tval;
	tval.tv_sec = millisec / 1000;
	tval.tv_usec = (millisec * 1000) % 1000000;
	select(0, NULL, NULL, NULL, &tval);
}

// --------------------------------------------------------
/// \概要:	获得参数
///
/// \参数:	w
/// \参数:	h
///
/// \返回:	FaceSDKConfig
// --------------------------------------------------------
FaceSDKConfig getConfig(int w, int h)
{
	FaceSDKConfig config;
	config.img_w = w;
	config.img_h = h;
	config.screen_w = w;
	config.screen_h = h;
	config.input_format = ImageFormat::BGR;
	config.mode = FaceSDKMode::Normal;
	config.thread_num = 2;
	return config;
}

// --------------------------------------------------------
/// \概要:	给图片添加文字
///
/// \参数:	image
/// \参数:	text
/// \参数:	origin
// --------------------------------------------------------
void Add_text_to_pic(cv::Mat &image, std::string text, cv::Point origin)
{
	int fontHeight = 50;
	int thickness = -1;
	int linestyle = 8;
	int baseline = 0;

	cv::Ptr<cv::freetype::FreeType2> ft2;
	ft2 = cv::freetype::createFreeType2();
	ft2->loadFontData(ch_ttf_, 0);

	ft2->putText(image, text, origin, fontHeight, cv::Scalar(0, 0, 255), thickness, linestyle, true);
}

// --------------------------------------------------------
/// \概要:	赋值人脸图片结构体
///
/// \参数:	face_msg
/// \参数:	msg
// --------------------------------------------------------
void Face_struct_assignment(face_plate_msgs::Face_pic &face_msg, const face_plate_msgs::Face_pic::ConstPtr msg)
{
	face_msg.vin = msg->vin;
	face_msg.deviceId = msg->deviceId;
	face_msg.pictureType = msg->pictureType;
	face_msg.sex = msg->sex;
	face_msg.age = msg->age;
	face_msg.facialExpression = msg->facialExpression;
	face_msg.race = msg->race;
	face_msg.hat = msg->hat;
	face_msg.bmask = msg->bmask;
	face_msg.eyeglass = msg->eyeglass;
	face_msg.capTime = msg->capTime;
	face_msg.facePicture = msg->facePicture;
	face_msg.faceScenePicture = msg->faceScenePicture;
}

// --------------------------------------------------------
/// \概要:	人脸照片回调函数
///
/// \参数:	msg
// --------------------------------------------------------
void FacePicMsgCallback(const face_plate_msgs::Face_pic::ConstPtr &msg)
{
	face_plate_msgs::Face_pic face_msg;
	Face_struct_assignment(face_msg, msg);
	cv::Mat frame = Base_to_Mat(face_msg.facePicture);

	int w = frame.cols % 2 + frame.cols;
	int h = frame.rows % 2 + frame.rows;

	cv::resize(frame, frame, cv::Size(w, h));
	FaceSDKConfig config = getConfig(w, h);
	facesdk_init(config);

	char data[w * h * 3];
	memcpy(data, (char *)frame.data, w * h * 3);
	facesdk_readModelFromFile(ModelType::Detect, model_detect_.c_str(), ImageFormat::RGB);
	sdkFaces faces = facesdk_detect(data);

	facesdk_readModelFromFile(ModelType::Attribution, model_attribution_.c_str(), ImageFormat::RGB);
	sdkFaces faces3 = facesdk_attribute();
	std::string text;

	if(faces3.face_count > 0)
	{
		if(faces3.info[0].attribution.gender == 0)
		{
			text = std::string("年龄：") + std::to_string(faces3.info[0].attribution.age) + std::string(" 性别： M");
			face_msg.sex = 1;
		}
		else if(faces3.info[0].attribution.gender == 1)
		{
			text = std::string("年龄：") + std::to_string(faces3.info[0].attribution.age) + std::string(" 性别： W");
			face_msg.sex = 2;
		}

		face_msg.age = int(faces3.info[0].attribution.age);
		std::cout << text << std::endl;

		pub_face_pic_message_.publish(face_msg);
	}

	//	cv::Point origin(faces.info[i].face_box.x1, faces.info[i].face_box.y1 - 60);
//	Add_text_to_pic(frame, text, origin);
#if 0
	sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
	pub_image_.publish(img_msg);
#endif
}



// --------------------------------------------------------
/// \概要:	主函数
///
/// \参数:	argc
/// \参数:	argv[]
///
/// \返回:	int
// --------------------------------------------------------
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "forecast_age_gender");
	ros::NodeHandle nh_("~");
	ros::Time time = ros::Time::now();

	nh_.param("/model_detect", model_detect_, std::string("./models/mtcnn_frozen_model.pb"));
	nh_.param("/model_attribution", model_attribution_, std::string("./models/mtcnn_frozen_model.pb"));
	nh_.param("/ch_ttf", ch_ttf_, std::string("./output_image/output_image01.jpg"));


#if 0
	image_transport::ImageTransport it(nh_);
	pub_image_ = it.advertise("/camera/image_" + cam_, 1);
#endif

	sub_pic_ = nh_.subscribe("/face_pic_msg", 1, FacePicMsgCallback);
	pub_face_pic_message_ = nh_.advertise<face_plate_msgs::Face_pic>("/face_attribute_msg", 1);

	ros::spin();

	return 0 ;
}
