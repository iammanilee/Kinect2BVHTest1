// Kinect2BVHTest1.cpp : 콘솔 응용 프로그램에 대한 진입점을 정의합니다.
//

#include "stdafx.h"
#include <fstream>
#include <iostream>
#include <string>

#include "bvhexport.h"


struct sKinectPosition
{
	int JointType;
	Vector4 Position;
};

struct sKinectRotation
{
	int JointType;
	Vector4 Quaternion;
};

struct sKinectFrame
{
	DWORD MilliSecond;
	sKinectPosition Pos;
	sKinectRotation Rot;
};

int main()
{
	std::string content;
	std::ifstream myfile;

	CBVH bvh;

	//bvh.ImportRefPoseByBVHFile2("Girl Blendswap5_AddRoot3.bvh");

	bvh.ImportRefPoseByBVHFile("Girl Blendswap5_AddRoot3.bvh");
	//bvh.SetKinectBoneConfiguration();

	myfile.open("rawtest.txt", std::ios::in);

	do
	{
		myfile >> content;

		if (content != "")
		{
			DWORD milliSeconds = std::stoi(content);

			bvh.Begin(milliSeconds);

			myfile >> content;
			if (content != "Pos")
				break;

			myfile >> content;
			int posCount = std::stoi(content);
			if (posCount == 0)
				break;

			for (int i = 0; i < posCount; ++i)
			{
				JointType jointType;
				float x, y, z;
				myfile >> content;
				jointType = (JointType)std::stoi(content);

				myfile >> content;
				x = std::stof(content);
				myfile >> content;
				y = std::stof(content);
				myfile >> content;
				z = std::stof(content);

				XMVECTOR position = { x, y, z };
				bvh.AddJointPositionValue(jointType, position);
			}


			myfile >> content;
			if (content != "Rot")
				break;

			myfile >> content;
			int rotCount = std::stoi(content);
			if (rotCount == 0)
				break;

			for (int i = 0; i < posCount; ++i)
			{
				JointType jointType;
				float x, y, z, w;
				myfile >> content;
				jointType = (JointType)std::stoi(content);

				myfile >> content;
				x = std::stof(content);
				myfile >> content;
				y = std::stof(content);
				myfile >> content;
				z = std::stof(content);
				myfile >> content;
				w = std::stof(content);

				XMVECTOR quat = { x,y,z,w };
				bvh.AddJointRotationValue(jointType, quat);
			}
			bvh.End();
		}

	} while (content != "");

	myfile.close();

	bvh.ExportFile("test.bvh");

    return 0;
}

