#include "StereoVision.h"

using namespace cv;

CStereoVision::CStereoVision()
{
	camsOpened = false;
	settingsLoaded = false;
}

CStereoVision::~CStereoVision()
{

}

int CStereoVision::initStereoVision(char* settingsPath, char* filterParamsPath, int leftCamID = -1, int rightCamID = -1)
{
	if (loadSettings(settingsPath) != 1)
		return 0;		
	if (loadFilter(filterParamsPath) != 1)
		return 0;
	if (openCameras(leftCamID, rightCamID) != 1)
		return 0;
	else
		camsOpened = true;
	
	settingsLoaded = true;

	return 1;
}

int CStereoVision::loadSettings(char* path)
{
	FileStorage fileStream;
	fileStream.open(path, FileStorage::READ);
	if (!fileStream.isOpened())
	{
		std::cout << "Nie udalo sie otworzyc pliku z parametrami kamer" << std::endl;
		return 0;
	}

	fileStream["leftCameraMat"] >> leftCameraMat;
	fileStream["leftCameraDistorsion"] >> leftCameraDistorsion;
	fileStream["rightCameraMat"] >> rightCameraMat;
	fileStream["rightCameraDistorsion"] >> rightCameraDistorsion;
	fileStream["rotationMat"] >> rotationMat;
	fileStream["leftRectificationMat"] >> leftRectificationMat;
	fileStream["leftProjectionMat"] >> leftProjectionMat;
	fileStream["rightRectificationMat"] >> rightRectificationMat;
	fileStream["rightProjectionMat"] >> rightProjectionMat;
	fileStream["disparity2DepthMat"] >> disparityToDepthMat;
	fileStream["leftValidPixROI"] >> leftValidPixROI;
	fileStream["rightValidPixROI"] >> rightValidPixROI;
	fileStream["imageSize"] >> imageSize;
	fileStream.release();

	return 1;
}

int CStereoVision::loadFilter(char* path)
{
	FileStorage fileStream;

	fileStream.open(path, FileStorage::READ);
	if (!fileStream.isOpened())
	{
		std::cout << "Nie udalo sie otworzyc pliku z parametrami filtrowania" << std::endl;
		return 0;
	}

	fileStream["method"] >> filterMethod;
	fileStream["min1"] >> filterMins[0];
	fileStream["min2"] >> filterMins[1];
	fileStream["min3"] >> filterMins[2];
	fileStream["max1"] >> filterMaxs[0];
	fileStream["max2"] >> filterMaxs[1];
	fileStream["max3"] >> filterMaxs[2];
	fileStream.release();

	return 1;
}

int CStereoVision::openCameras(int leftCamID, int rightCamID)
{
	leftCam.open(leftCamID);
	if (!leftCam.isOpened())
	{
		std::cout << "Nie udalo sie uruchomic kamery: " << leftCamID << std::endl;
		return 0;
	}
	rightCam.open(rightCamID);
	if (!rightCam.isOpened())
	{
		std::cout << "Nie udalo sie uruchomic kamery: " << rightCamID << std::endl;
		return 0;
	}
	
	return 1;
}

int CStereoVision::closeCameras()
{
	if (leftCam.isOpened())
		leftCam.release();
	if (rightCam.isOpened())
		rightCam.release();
	camsOpened = false;

	return 1;
}

int CStereoVision::grabFrames()
{
	if (!camsOpened)
	{
		std::cout << "Kamery nie sa uruchomione" << std::endl;
		return 0;
	}
	leftCam >> leftFrame;
	rightCam >> rightFrame;

	return 1;
}

void CStereoVision::filterFrames(cv::Mat& left, cv::Mat& right, int method)
{
	if (method == HSV)
	{
		cvtColor(leftFrame, leftFrame, CV_BGR2HSV);
		cvtColor(rightFrame, rightFrame, CV_BGR2HSV);
	}
	inRange(leftFrame, Scalar(filterMins[0], filterMins[1], filterMins[2]), Scalar(filterMaxs[0], filterMaxs[1], filterMaxs[2]), leftFilteredFrame);
	inRange(rightFrame, Scalar(filterMins[0], filterMins[1], filterMins[2]), Scalar(filterMaxs[0], filterMaxs[1], filterMaxs[2]), rightFilteredFrame);

}

int CStereoVision::undistortRectifyFrames(Mat &leftImage, Mat &rightImage)
{
	Mat leftMapX, leftMapY, rightMapX, rightMapY;

	initUndistortRectifyMap(leftCameraMat, leftCameraDistorsion, leftRectificationMat, leftProjectionMat, imageSize, CV_32F, leftMapX, leftMapY);
	initUndistortRectifyMap(rightCameraMat, rightCameraDistorsion, rightRectificationMat, rightProjectionMat, imageSize, CV_32F, rightMapX, rightMapY);

	remap(leftImage, leftTransformedFrame, leftMapX, leftMapY, INTER_LINEAR);
	remap(rightImage, rightTransformedFrame, rightMapX, rightMapY, INTER_LINEAR);

	return 1;
}

void CStereoVision::showImage(Mat image, bool waitForKey)
{
	namedWindow("window");
	imshow("window", image);
	if (waitForKey)
		waitKey();
	destroyWindow("window");
}

void CStereoVision::showImage(char* windowName, Mat image, bool waitForKey = 0)
{
	imshow(windowName, image);
	if (waitForKey)
		waitKey();
}

void CStereoVision::drawParallelLines(Mat & image)
{
	Size imageSize = image.size();
	
	for (int i = 0; i < imageSize.height; i+=32)
	{
		line(image, Point(0, i), Point(imageSize.width, i), Scalar(0, 255, 0),1);
	}
}

Point2f CStereoVision::findPoint(Mat& img)
{
	float xMin = img.cols, xMax = 0, yMin = img.rows, yMax = 0;
	int counter = 0;
	uchar* pointer;
	for (int i = 0; i < img.rows; i++)
	{
		pointer = img.ptr(i);
		for (int j = 0; j < img.cols; j++)
		{
			if (pointer[j] == 255)
			{
				counter++;
				if (j < xMin)
					xMin = j;
				if (j > xMax)
					xMax = j;
				if (i < yMin)
					yMin = i;
				if (i > yMax)
					yMax = i;
			}
		}
	}

	if (counter == 1)	// jeden punkt odnaleziony
		return Point2f(xMax, yMax);
	else if (counter == 0)	// 0 punktow
		return Point2f(0,0);
	else				// wiele punktow - blop
		return Point2f(xMin + (xMax - xMin) / 2, yMin + (yMax - yMin) / 2);
}

Point3f CStereoVision::triangulate(Mat& leftImg, Mat& rightImg)
{
	std::vector<Point2f> leftPoint, rightPoint;
	Point2f left, right;
	Point3f point3D;
	Mat point4D = Mat(4, 1, CV_32F);

	left = findPoint(leftImg);
	right = findPoint(rightImg);
	if (left == Point2f(0, 0) || right == Point2f(0, 0))	// czyli brak punktow / (0,0)
		point3D = Point3f(0, 0, 0);
	else
	{
		leftPoint.push_back(left);
		rightPoint.push_back(right);

		triangulatePoints(leftProjectionMat, rightProjectionMat,
			leftPoint, rightPoint, point4D);

		point3D = calcPoint3D(point4D);
	}

	return point3D;
}

float CStereoVision::getPixelValue(Mat& img, int x, int y)
{
	float* ptr = img.ptr<float>(x - 1);
	return ptr[y - 1];
}

Point3f CStereoVision::calcPoint3D(Mat& point4D)
{
	Point3f point3D;
	if (getPixelValue(point4D, 3, 1) == 0)
		point3D = Point3f(0, 0, 0);
	else
	{
		float w = getPixelValue(point4D, 4, 1);
		point3D.x = getPixelValue(point4D, 1, 1) / w;
		point3D.y = getPixelValue(point4D, 2, 1) / w;
		point3D.z = getPixelValue(point4D, 3, 1) / w;
	}

	return point3D;
}

Point3f  CStereoVision::coordinateTransform(Point3f point, Point3f trans, Point3f rot)
{
	Mat rotXMat = Mat::eye(4, 4, CV_32F);
	Mat rotYMat = Mat::eye(4, 4, CV_32F);
	Mat rotZMat = Mat::eye(4, 4, CV_32F);
	Mat transMat = Mat::eye(4, 4, CV_32F);
	Mat invRotXMat, invRotYMat, invRotZMat, invTransMat;
	Mat cameraPoint = Mat(point);

	cameraPoint.resize(4);
	cameraPoint.at<float>(3, 0) = 1;

	transMat.at<float>(0, 3) = trans.x;
	transMat.at<float>(1, 3) = trans.y;
	transMat.at<float>(2, 3) = trans.z;

	rotXMat.at<float>(1, 1) = (float)cos(rot.x * PI / 180);
	rotXMat.at<float>(1, 2) = (float)-sin(rot.x * PI / 180);
	rotXMat.at<float>(2, 1) = (float)sin(rot.x * PI / 180);
	rotXMat.at<float>(2, 2) = (float)cos(rot.x * PI / 180);

	rotYMat.at<float>(0, 0) = (float)cos(rot.y * PI / 180);
	rotYMat.at<float>(0, 2) = (float)sin(rot.y * PI / 180);
	rotYMat.at<float>(2, 0) = (float)-sin(rot.y * PI / 180);
	rotYMat.at<float>(2, 2) = (float)cos(rot.y * PI / 180);

	rotZMat.at<float>(0, 0) = (float)cos(rot.z * PI / 180);
	rotZMat.at<float>(0, 1) = (float)-sin(rot.z * PI / 180);
	rotZMat.at<float>(1, 0) = (float)sin(rot.z * PI / 180);
	rotZMat.at<float>(1, 1) = (float)cos(rot.z * PI / 180);

	Mat result = transMat * rotXMat * rotYMat * rotZMat * cameraPoint;
	result.resize(3);

	return Point3f(result);
}