#pragma once
#include "opencv2\opencv.hpp"
#include <math.h>

#define PI 3.14159265358979323846
#define RGB 1
#define HSV 2

class CStereoVision
{
public:
	CStereoVision();
	~CStereoVision();

	int initStereoVision(char* path, char* filterParamsPath, int leftCamID, int rightCamID);
	int loadSettings(char* path);
	int loadFilter(char* path);
	int openCameras(int leftCamID, int rightCamID);
	int closeCameras();
	int grabFrames();
	int filterFrames(cv::Mat& left, cv::Mat& right);
	int filterFrames_BGR(cv::Mat& left, cv::Mat& right);
	int filterFrames_HSV(cv::Mat& left, cv::Mat& right);
	int undistortRectifyFrames(cv::Mat &leftFrame, cv::Mat &rightFrame);
	void showImage(cv::Mat image, bool waitForKey);
	void showImage(char* windowName, cv::Mat image, bool waitForKey);
	void drawParallerLines(cv::Mat &image);
	cv::Point2f findPoint(cv::Mat& img);
	float getPixelValue(cv::Mat& img, int x, int y);
	cv::Point3f calcPoint3D(cv::Mat& point4D);
	cv::Point3f triangulate(cv::Mat& leftImg, cv::Mat& rightImg);
	cv::Point3f coordinateTransform(cv::Point3f point, cv::Point3f trans, cv::Point3f rot);


	bool settingsLoaded;
	bool camsOpened;
	int filterMethod;
	int filterMins[3];
	int filterMaxs[3];
	cv::VideoCapture leftCam, rightCam;
	cv::Mat leftCameraMat, leftCameraDistorsion, rightCameraMat, rightCameraDistorsion;
	cv::Mat rotationMat, leftRectificationMat, leftProjectionMat,
					rightRectificationMat, rightProjectionMat;
	cv::Mat disparityToDepthMat;
	cv::Mat leftFrame, rightFrame;
	cv::Mat leftFilteredFrame, rightFilteredFrame;
	cv::Mat leftTransformedFrame, rightTransformedFrame;
	cv::Mat disparityMap;
	cv::Rect leftValidPixROI, rightValidPixROI;
	cv::Size imageSize;
	cv::Point3f coordnateTrans;
	cv::Point3f coordnateRot;
};

