#define NOMINMAX

#include <iostream>
#include <fstream>
#include <string>
#include <opencv2\opencv.hpp>
#include "StereoVision.h"
#include "TCPConnection.h"

#define KAWASAKI_ADDRESS "11.12.1.30"
#define KAWASAKI_PORT "9001"
#define MIN_POINTS 3
#define FIRST_POINTS_IGNORE 2

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

int loadCordTransformation(char* path, Point3f &trans, Point3f &rot)
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

bool checkRange(int number, int min, int max)
{
	if (number >= min && number <= max)
		return true;
	else
	{
		cout << "Liczba musi sie zawierac w przedziale: <" << min << ";" << max << ">\n";
		return false;
	}
}

int inputNumber(int minNumber, int maxNumber)
{
	int number;
	do
	{
		cin >> number;
		while (cin.fail())
		{
			cin.clear();
			cin.ignore(numeric_limits<std::streamsize>::max(), '\n');
			cout << "Zla liczba, wprowadz ponownie: ";
			cin >> number;
		}
	} while (!checkRange(number, minNumber, maxNumber));

	return number;
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
	plik.open("pomiarX_skoki.txt", std::ios::out);
	CStereoVision stereoVision;
	Mat detectedPoint4D;
	Point3f detectedPoint3D, coordsTrans, coordsRot;
	CTCPConnection robotConnection;
	vector<Point3f> points;
	int firstPointsToIgnore = 0;
	int leftID, rightID;
	string robotAddress, robotPort;
	namedWindow("leftCam");
	namedWindow("rightCam");

	do
	{
		cout << "Podaj ID lewej kamery: ";
		leftID = inputNumber(0, 10);
		cout << "Podaj ID prawej kamery: ";
		rightID = inputNumber(0, 10);
	} while (stereoVision.initStereoVision("calibrationParameters.xml", "filterParameters.xml", leftID, rightID) != 1);
	
	if (!loadCordTransformation("coordinateTransformation.xml", coordsTrans, coordsRot))
		return 0;
	cout << "Wczytano dane z plikow\n";
	/*
	do
	{
		cout << "Podaj adres IP robota: ";
		cin >> robotAddress;
		cout << "Podaj port robota: ";
		cin >> robotPort;
	} while (!robotConnection.setupConnection(robotAddress.c_str(), robotPort.c_str()));
	
	
	if (!robotConnection.setupConnection(KAWASAKI_ADDRESS, KAWASAKI_PORT))
		return 0;
	*/
	
	while ((waitKey(3) == -1))
	{
		stereoVision.grabFrames();
		stereoVision.undistortRectifyFrames(stereoVision.leftFrame, stereoVision.rightFrame);
		stereoVision.drawParallelLines(stereoVision.leftTransformedFrame);
		stereoVision.drawParallelLines(stereoVision.rightTransformedFrame);
		imshow("leftCam", stereoVision.leftTransformedFrame);
		imshow("rightCam", stereoVision.rightTransformedFrame);
	}
	
	while ((waitKey(3) == -1))
	{
		stereoVision.grabFrames();
		stereoVision.undistortRectifyFrames(stereoVision.leftFrame, stereoVision.rightFrame);
		stereoVision.filterFrames(stereoVision.leftTransformedFrame, stereoVision.rightTransformedFrame, stereoVision.filterMethod);
		imshow("leftCam", stereoVision.leftFilteredFrame);
		imshow("rightCam", stereoVision.rightFilteredFrame);
		detectedPoint3D = stereoVision.triangulate(stereoVision.leftFilteredFrame, stereoVision.rightFilteredFrame);
		if (detectedPoint3D == Point3f(0, 0, 0))
		{
			points.clear();
			firstPointsToIgnore = 0;
			continue;
		}

		saveToFile(plik, detectedPoint3D);
		cout << detectedPoint3D << endl;
		/*
		if (firstPointsToIgnore < FIRST_POINTS_IGNORE)
		{
			firstPointsToIgnore++;
			continue;
		}
		
		//cout << detectedPoint3D << endl;
		points.push_back(detectedPoint3D);			
		if (points.size() >= MIN_POINTS && robotConnection.isConnected())
		{
			Point3f pointToSend = averagePoints(points);
			pointToSend = stereoVision.coordinateTransform(pointToSend, coordsTrans, coordsRot);	// punkt w odniesieniu do nowego ukl. wsp.
			points.clear();
			std::string dataToSend = std::to_string(pointToSend.x) + ";" +
				std::to_string(pointToSend.y) + ";" +
				std::to_string(pointToSend.z+150) + ";";
			if (robotConnection.sendData(dataToSend.c_str()) != 0)
			{
				cout << "WYSLANO:\n" << dataToSend.c_str() << endl;
			}
		}
		*/		
		
	}

	cv::waitKey();
	plik.close();

	return 1;
}