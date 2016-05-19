#include <iostream>
#include <fstream>
#include <string>
#include <opencv2\opencv.hpp>
#include "StereoVision.h"
#include "TCPConnection.h"

#define KAWASAKI_ADDRESS "11.12.1.30"
#define KAWASAKI_PORT "9001"
#define MIN_POINTS 4

using namespace cv;
using namespace std;

float getPixelValue(Mat& img, int x, int y)
{
	float* ptr = img.ptr<float>(x-1);
	return ptr[y-1];
}

void saveToFile(ofstream& file, Point3f& point)
{
	if (file.is_open())
	{
		file << point.x << ";"
			<< point.y << ";"
			<< point.z << "\n";
	}
}

int loadCordTransformation(char* path, Point3f trans, Point3f rot)
{
	FileStorage fileStream;
	fileStream.open(path, FileStorage::READ);
	if (!fileStream.isOpened())
	{
		std::cout << "Nie udalo sie otworzyc pliku z transformacja ukladu wspolrzednych" << std::endl;
		return 0;
	}

	fileStream["translationX"] >> trans.x;
	fileStream["translationY"] >> trans.y;
	fileStream["translationZ"] >> trans.z;
	fileStream["rotationX"] >> rot.x;
	fileStream["rotationY"] >> rot.y;
	fileStream["rotationZ"] >> rot.z;

	fileStream.release();

	return 1;
}

Point3f calcPoint3D(Mat& point4D)
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

Point3f averagePoints(vector<Point3f>& pointsVec)
{
	Point3f average;

	for (int i = 0; i < pointsVec.size(); i++)
	{
		average += pointsVec[i];
	}
	average.x /= pointsVec.size();
	average.y /= pointsVec.size();
	average.z /= pointsVec.size();

	return average;
}

/*bool checkWorkingZone(Point3f point)
{
	Point3f firstCorner(-686, 3, -100);
	Point3f secondCorner(-1083,-377,200);

	if (point.x >= firstCorner.x && point.x <= secondCorner.x)
	{
		if (point.y >= firstCorner.y && point.y <= secondCorner.y)
		{
			if (point.z >= firstCorner.z && point.z <= secondCorner.z)
				return true;
		}
	}

	return false;
}*/

int main()
{
	ofstream plik;
	plik.open("pomiarZ_skoki.txt", std::ios::out);
	CStereoVision stereoVision;
	Mat detectedPoint4D;
	Point3f detectedPoint3D, coordsTrans, coordsRot;
	CTCPConnection robotConnection;
	vector<Point3f> points;
	namedWindow("leftCam");
	//namedWindow("rightCam");

	if (stereoVision.initStereoVision("calibrationParameters.xml", "filterParameters.xml", 2, 1) != 1)
		return 0;
	/*
	if (!loadCordTransformation("coordinateTransformation.yml", coordsTrans, coordsRot))
		return 0;
	if (!robotConnection.setupConnection(KAWASAKI_ADDRESS, KAWASAKI_PORT))
		return 0;
	*/
	while ((waitKey(3) == -1))
	{
		stereoVision.grabFrames();
		stereoVision.undistortRectifyFrames(stereoVision.leftFrame, stereoVision.rightFrame);
		stereoVision.filterFrames_BGR(stereoVision.leftTransformedFrame, stereoVision.rightTransformedFrame);
		imshow("leftCam", stereoVision.leftFilteredFrame);
		//imshow("rightCam", stereoVision.rightFrame);
		detectedPoint3D = stereoVision.triangulate(stereoVision.leftFilteredFrame, stereoVision.rightFilteredFrame);
		if (detectedPoint3D == Point3f(0, 0, 0))
		{
			//points.clear();
			continue;
		}	
		/*
		detectedPoint3D = stereoVision.coordinateTransform(detectedPoint3D, Point3f(-1000, -1000, 800), Point3f(-135, 0, 0));	// punkt w odniesieniu do nowego ukl. wsp.
		points.push_back(detectedPoint3D);			
		if (points.size() >= MIN_POINTS && robotConnection.isConnected())
		{
			Point3f pointToSend = averagePoints(points);
			points.clear();
			cout << "WYSYLAM...\n";
			std::string dataToSend = std::to_string(pointToSend.x) + ";" +
				std::to_string(pointToSend.y) + ";" +
				std::to_string(pointToSend.z + 50) + ";";
			if (robotConnection.sendData(dataToSend.c_str()) != 0)
			{
				cout << "WYSLANO:\n" << dataToSend.c_str() << endl;
			}
		}
		*/
		cout << detectedPoint3D << endl;		
		saveToFile(plik, detectedPoint3D);
	}

	cv::waitKey();
	plik.close();

	return 1;
}