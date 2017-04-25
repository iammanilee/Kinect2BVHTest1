#include "stdafx.h"

#include <assert.h>
#include <string>
#include <list>

#include <iostream>
#include <fstream>

#include "bvhexport.h"
#include "quaternion.h"

void QuaternionToEulerAngles(const XMVECTOR& inQuat, XMVECTOR& outEulerianAngles)
{
	double eulerxyz[3];
	Quaternion quaternion = inQuat;
	quaternion.normalize();
	quaternion2Euler(quaternion, eulerxyz, zyx);

	outEulerianAngles = { (float)eulerxyz[0], (float)eulerxyz[1], (float)eulerxyz[2], 0.0f };
}

void CBVH::ResetJointParentIndex()
{
	if (RootJoint)
	{ 
		delete RootJoint;
		RootJoint = nullptr;
	}

	std::unordered_map<std::string, FBVHJoint*> jointNameMap;

	int index = 0;
	for (auto const& value : JointNames)
	{
		FBVHJoint* bvhJoint = new FBVHJoint;
		bvhJoint->JointName = value;
		bvhJoint->JointIndex = index;
		jointNameMap.insert(std::pair<std::string, FBVHJoint*>(value, bvhJoint));
		index++;
	}

	auto iter = JointNames.begin();
	auto piter = JointParentNames.begin();
	auto biter = JointBoneDirections.begin();
	for (; iter != JointNames.end(); iter++, piter++, biter++)
	{
		auto const& jointName = *iter;
		auto const& jointParentName = *piter;
		auto jointBoneDirection = *biter;

		auto iterBVHJoint = jointNameMap.find(jointName);
		auto iterParentBVHJoint = jointNameMap.find(jointParentName);

		if (iterParentBVHJoint == jointNameMap.end())
		{ 
			assert(RootJoint == nullptr);

			// root joint
			RootJoint = iterBVHJoint->second;

			continue;
		}

		auto bvhJoint = iterBVHJoint->second;
		auto parentBVHJoint = iterParentBVHJoint->second;

		bvhJoint->ParentJoint = parentBVHJoint;
		bvhJoint->BoneDirection = jointBoneDirection;
		parentBVHJoint->ChildrenJoint.push_back(bvhJoint);
	}

	assert(RootJoint != nullptr);

	SortedJointArray.clear();
	RootJoint->GatherJoints(SortedJointArray);

	IndexConvertTable.resize(JointCount);

	index = 0;
	for (auto const& value : SortedJointArray)
	{
		IndexConvertTable[value->JointIndex] = index;
		index++;
	}
}

void CBVH::GenerateLocalRotation()
{
	// SortedJointArray, RawFrames  두 순서는 동일하다.

	int frameIndex = 0;
	for (auto& frame : RawFrames)
	{
		int index = 0;
		for (auto& frameInfo : frame.FrameInfo)
		{
			FBVHJoint* bvhJoint = SortedJointArray[index];
			if (bvhJoint)
			{
				if (bvhJoint->ParentJoint)
				{
					auto parentIndex = bvhJoint->ParentJoint->JointIndex;
					auto& parentFrameInfo = frame.FrameInfo[parentIndex];

					assert(index > parentIndex);

					if (frameInfo.Initialized)
					{
						// local*parent.world = world
						// local = world*inverse(parent.world)
						frameInfo.WorldQuat = XMQuaternionNormalize(frameInfo.WorldQuat);
						frameInfo.LocalQuat = XMQuaternionMultiply(frameInfo.WorldQuat, XMQuaternionInverse(parentFrameInfo.WorldQuat));
						frameInfo.LocalQuat = XMQuaternionNormalize(frameInfo.LocalQuat);

						if (XMVectorGetX(frameInfo.LocalQuat) == 0.0f &&
							XMVectorGetY(frameInfo.LocalQuat) == 0.0f &&
							XMVectorGetZ(frameInfo.LocalQuat) == 0.0f &&
							XMVectorGetW(frameInfo.LocalQuat) == 0.0f)
						{
							int a = 0;
						}
					}
					else
					{
						frameInfo.LocalQuat = bvhJoint->RefQuat;
						frameInfo.WorldQuat = frameInfo.LocalQuat*parentFrameInfo.WorldQuat;
					}
				}
				else
				{
					if (frameInfo.Initialized)
					{
						frameInfo.WorldQuat = XMQuaternionNormalize(frameInfo.WorldQuat);
						frameInfo.LocalQuat = XMQuaternionNormalize(frameInfo.WorldQuat);
					}
					else
					{
						frameInfo.WorldQuat = bvhJoint->RefQuat;
						frameInfo.LocalQuat = bvhJoint->RefQuat;
					}
				}
			}

			//frameInfo.LocalQuat = frameInfo.WorldQuat;

			++index;
		}

		frameIndex++;
	}
}

void CBVH::GenerateEvenSpacedFrameData()
{
	if (RawFrames.size() < 2)
		return;

	Frames.clear();

	int currentFrameIndex = 0;
	DWORD initialFrameTime = RawFrames[0].ElapseTime;
	DWORD currentFrameTime = RawFrames[0].ElapseTime;

	for (size_t i = 0; i < RawFrames.size() - 1; ++i)
	{
		auto& rawframe0 = RawFrames[i];
		auto& rawframe1 = RawFrames[i + 1];

		auto frameCount = Frames.size();

		if (rawframe0.ElapseTime == currentFrameTime)
		{
			Frames.resize(frameCount + 1);

			Frames[frameCount].ElapseTime = currentFrameTime - initialFrameTime;
			Frames[frameCount].FrameInfo = rawframe0.FrameInfo;

			for (size_t j = 0; j < Frames[frameCount].FrameInfo.size(); ++j)
			{
				auto& value = Frames[frameCount].FrameInfo[j];



				// deviation from refPose
				auto& joint = SortedJointArray[j];

				// localQuat = devQuat*refPoseQuat;
				// devQuat = localQuat*inverse(refPoseQuat)
				value.LocalQuat = rawframe0.FrameInfo[j].LocalQuat;
				value.DevQuat = XMQuaternionMultiply(value.LocalQuat, joint->InvRefQuat);

				// quaternion to eulerian angles
				QuaternionToEulerAngles(value.DevQuat, value.Rotation);
				//QuaternionToEulerAngles(value.LocalQuat, value.Rotation);
			}

			++currentFrameIndex;
			currentFrameTime = initialFrameTime + (currentFrameIndex) * 1000 / ExportFrameRate;
		}


		while(rawframe0.ElapseTime <= currentFrameTime && currentFrameTime < rawframe1.ElapseTime)
		{
			auto frameCount = Frames.size();

			Frames.resize(frameCount + 1);

			float interpTime = (float)(currentFrameTime - rawframe0.ElapseTime)/ (float)(rawframe1.ElapseTime - rawframe0.ElapseTime);
			
			auto& dest = Frames[frameCount].FrameInfo;
			Frames[frameCount].ElapseTime = currentFrameTime - initialFrameTime;

			for (size_t j = 0; j < rawframe0.FrameInfo.size(); ++j)
			{
				auto& rawJoint0 = rawframe0.FrameInfo[j];
				auto& rawJoint1 = rawframe1.FrameInfo[j];

				FBVHJointTransform interpTransform;
				interpTransform.LocalQuat = XMQuaternionSlerp(rawJoint0.LocalQuat, rawJoint1.LocalQuat, interpTime);

				// deviation from refPose
				auto& joint = SortedJointArray[j];

				// localQuat = devQuat*refPoseQuat;
				// devQuat = localQuat*inverse(refPoseQuat)
				interpTransform.DevQuat = XMQuaternionMultiply(interpTransform.LocalQuat, joint->InvRefQuat);

				// quaternion to eulerian angles
				QuaternionToEulerAngles(interpTransform.DevQuat, interpTransform.Rotation);
				//QuaternionToEulerAngles(interpTransform.LocalQuat, interpTransform.Rotation);
				dest.push_back(interpTransform);
			}

			++currentFrameIndex;
			currentFrameTime = initialFrameTime + (currentFrameIndex) * 1000 / ExportFrameRate;
		}
	}
}

void CBVH::MakeNameJointTypeMap()
{
	NameJointTypeMap["SpineBase"] = JointType_SpineBase;
	NameJointTypeMap["SpineMid"] = JointType_SpineMid;
	NameJointTypeMap["Neck"] = JointType_Neck;
	NameJointTypeMap["Head"] = JointType_Head;
	NameJointTypeMap["ShoulderLeft"] = JointType_ShoulderLeft;
	NameJointTypeMap["ElbowLeft"] = JointType_ElbowLeft;
	NameJointTypeMap["WristLeft"] = JointType_WristLeft;
	NameJointTypeMap["HandLeft"] = JointType_HandLeft;
	NameJointTypeMap["ShoulderRight"] = JointType_ShoulderRight;
	NameJointTypeMap["ElbowRight"] = JointType_ElbowRight;
	NameJointTypeMap["WristRight"] = JointType_WristRight;
	NameJointTypeMap["HandRight"] = JointType_HandRight;
	NameJointTypeMap["HipLeft"] = JointType_HipLeft;
	NameJointTypeMap["KneeLeft"] = JointType_KneeLeft;
	NameJointTypeMap["AnkleLeft"] = JointType_AnkleLeft;
	NameJointTypeMap["FootLeft"] = JointType_FootLeft;
	NameJointTypeMap["HipRight"] = JointType_HipRight;
	NameJointTypeMap["KneeRight"] = JointType_KneeRight;
	NameJointTypeMap["AnkleRight"] = JointType_AnkleRight;
	NameJointTypeMap["FootRight"] = JointType_FootRight;
	NameJointTypeMap["SpineShoulder"] = JointType_SpineShoulder;
	NameJointTypeMap["HandTipLeft"] = JointType_HandTipLeft;
	NameJointTypeMap["ThumbLeft"] = JointType_ThumbLeft;
	NameJointTypeMap["HandTipRight"] = JointType_HandTipRight;
	NameJointTypeMap["ThumbRight"] = JointType_ThumbRight;
}

JointType CBVH::GetJointType(const std::string & inJointName)
{
	auto iter = NameJointTypeMap.find(inJointName);
	if (iter != NameJointTypeMap.end())
	{
		return iter->second;
	}

	return _JointType();
}

CBVH::CBVH() : NumberOfFrames(0), NumberOfFramesInSecond(0), CurrentElapseTime(INVALID_ELAPSE_TIME), JointCount(0), RootJoint(nullptr), CurrentRawBVHFrame(nullptr)
{

}

CBVH::~CBVH()
{
	if (RootJoint)
	{
		delete RootJoint;
	}
}

void CBVH::Begin(DWORD inMilliSeconds)
{
	CurrentElapseTime = inMilliSeconds;

	auto count = RawFrames.size();
	RawFrames.resize(count + 1);

	CurrentRawBVHFrame = &RawFrames[count];
	CurrentRawBVHFrame->ElapseTime = CurrentElapseTime;
	CurrentRawBVHFrame->FrameInfo.resize(JointCount);
}

void CBVH::AddJointRotationValue(JointType inKinectJointType, const XMVECTOR& inQuat)
{
	if (CurrentRawBVHFrame && inKinectJointType < JointType_Count)
	{
		int SortedIndex = IndexConvertTable[inKinectJointType];

		if (XMVectorGetX(inQuat) == 0.0f &&
			XMVectorGetY(inQuat) == 0.0f &&
			XMVectorGetZ(inQuat) == 0.0f &&
			XMVectorGetW(inQuat) == 0.0f)
		{
			CurrentRawBVHFrame->FrameInfo[SortedIndex].Initialized = false;
			return;
		}

		CurrentRawBVHFrame->FrameInfo[SortedIndex].Initialized = true;
		CurrentRawBVHFrame->FrameInfo[SortedIndex].WorldQuat = inQuat;

		assert(GetJointType(SortedJointArray[SortedIndex]->JointName) == inKinectJointType);
	}
}

void CBVH::AddJointPositionValue(JointType inKinectJointType, const XMVECTOR& inPosition)
{
	if (CurrentRawBVHFrame && inKinectJointType < JointType_Count)
	{
		int SortedIndex = IndexConvertTable[inKinectJointType];
		CurrentRawBVHFrame->FrameInfo[SortedIndex].Initialized = true;
		CurrentRawBVHFrame->FrameInfo[SortedIndex].Position = inPosition;

		assert(GetJointType(SortedJointArray[SortedIndex]->JointName) == inKinectJointType);
	}
}

void CBVH::End()
{
	CurrentElapseTime = INVALID_ELAPSE_TIME;
}

void CBVH::ImportRefPoseByBVHFile(const std::string & inFileName)
{
	MakeNameJointTypeMap();

	// ROOT, JOINT
	std::string content;
	std::ifstream myfile;

	myfile.open(inFileName, std::ios::in);

	std::list<FBVHJoint*> jointStack;
	FBVHJoint* currentJoint = NULL;

	int jointIndex = 0;

	do
	{
		myfile >> content;

		if (content == "ROOT")
		{
			currentJoint = new FBVHJoint;
			RootJoint = currentJoint;
			myfile >> content; currentJoint->JointName = content;

			currentJoint->JointIndex = jointIndex++;
			currentJoint->KinectJointType = GetJointType(currentJoint->JointName);
		}
		else if (content == "JOINT")
		{
			FBVHJoint* parentJoint = currentJoint;
			if (parentJoint)
			{
				jointStack.push_back(currentJoint);
			}
			else
			{
				parentJoint = jointStack.back();
			}

			currentJoint = new FBVHJoint(parentJoint);
			if (parentJoint)
			{
				parentJoint->ChildrenJoint.push_back(currentJoint);
			}

			currentJoint->JointIndex = jointIndex++;

			myfile >> content;
			currentJoint->JointName = content;
			currentJoint->KinectJointType = GetJointType(currentJoint->JointName);
		}
		else if (content == "OFFSET")
		{
			float x, y, z;
			myfile >> content; x = (float)std::atof(content.c_str());
			myfile >> content; y = (float)std::atof(content.c_str());
			myfile >> content; z = (float)std::atof(content.c_str());

			if (currentJoint)
			{
				currentJoint->Position = { x, y, z };
			}
		}
		else if (content == "ROT")
		{
			float x, y, z, w;
			myfile >> content; x = (float)std::atof(content.c_str());
			myfile >> content; y = (float)std::atof(content.c_str());
			myfile >> content; z = (float)std::atof(content.c_str());
			myfile >> content; w = (float)std::atof(content.c_str());

			if (currentJoint)
			{
				currentJoint->RefQuat = { x, y, z, w };

				currentJoint->RefQuat = XMQuaternionNormalize(currentJoint->RefQuat);
				currentJoint->InvRefQuat = XMQuaternionInverse(currentJoint->RefQuat);
			}
		}
		else if (content == "EULER")
		{
			float x, y, z;
			myfile >> content; x = (float)std::atof(content.c_str());
			myfile >> content; y = (float)std::atof(content.c_str());
			myfile >> content; z = (float)std::atof(content.c_str());

			currentJoint->RefEuler = { x, y, z };
		}
		else if (content == "End")
		{
			FBVHJoint* parentJoint = currentJoint;
			if (parentJoint)
			{
				jointStack.push_back(currentJoint);
			}
			else
			{
				parentJoint = jointStack.back();
			}

			currentJoint = new FBVHJoint(parentJoint);
			if (parentJoint)
			{
				parentJoint->ChildrenJoint.push_back(currentJoint);
			}

			currentJoint->JointIndex = -1;

			myfile >> content;
			currentJoint->JointName = content;

			//myfile >> content;		// Site
			//myfile >> content;		// {
			//myfile >> content;		// OFFSET
			//myfile >> content;		// X
			//myfile >> content;		// Y
			//myfile >> content;		// Z
			//myfile >> content;		// }
		}
		else if (content == "}")
		{
			if (currentJoint)
				currentJoint = nullptr;
			else
			{
				jointStack.pop_back();
			}
		}
	} while (content != "MOTION");

	myfile.close();

	SortedJointArray.clear();
	RootJoint->GatherJoints(SortedJointArray);
	JointCount = (int)SortedJointArray.size();

	// Kinect 의 JointType을  JointArray Index로 변환
	IndexConvertTable.resize(JointCount);

	for (auto const& value : SortedJointArray)
	{
		IndexConvertTable[value->KinectJointType] = value->JointIndex;

		OutputDebugStringA(value->JointName.c_str());
		OutputDebugStringA(" ");
		OutputDebugStringA(std::to_string(value->JointIndex).c_str());
		OutputDebugStringA("\n");
	}
}

void CBVH::ImportRefPoseByBVHFile2(const std::string & inFileName)
{
	std::string content;
	std::ifstream myfile;

	myfile.open(inFileName, std::ios::in);

	XMVECTOR rot;
	XMVECTOR euler;

	do
	{
		myfile >> content;

		if (content == "ROOT")
		{
			myfile >> content;
		}
		else if (content == "JOINT")
		{
			myfile >> content;
		}
		else if (content == "OFFSET")
		{
			myfile >> content;
			myfile >> content;
			myfile >> content;
		}
		else if (content == "ROT")
		{
			float x, y, z, w;
			myfile >> content; x = (float)std::atof(content.c_str());
			myfile >> content; y = (float)std::atof(content.c_str());
			myfile >> content; z = (float)std::atof(content.c_str());
			myfile >> content; w = (float)std::atof(content.c_str());

			rot = { x, y, z, w };
		}
		else if (content == "EULER")
		{
			float x, y, z;
			myfile >> content; x = (float)std::atof(content.c_str());
			myfile >> content; y = (float)std::atof(content.c_str());
			myfile >> content; z = (float)std::atof(content.c_str());

			euler = { x, y, z, 0.0f };

			XMVECTOR convertedEuler = { 0.0f, 0.0f, 0.0f, 0.0f };
			QuaternionToEulerAngles(rot, convertedEuler);

			double eulerxyz[3];
			Quaternion quaternion = rot;
			quaternion.normalize();
			quaternion2Euler(quaternion, eulerxyz, zyx);

			if (abs(XMVectorGetX(euler) - XMVectorGetX(convertedEuler)) > 0.1f ||
				abs(XMVectorGetY(euler) - XMVectorGetY(convertedEuler)) > 0.1f ||
				abs(XMVectorGetZ(euler) - XMVectorGetZ(convertedEuler)) > 0.1f)
			{
				int a = 0;
			}
		}
		else if (content == "End")
		{
			myfile >> content;
		}
		else if (content == "}")
		{
		}
	} while (content != "MOTION");

	myfile.close();
}

void CBVH::DataValidationTest()
{
	///////////////////////////////////////
	// Reference pose로 받은 값을 검증
	///////////////////////////////////////

	for (auto const& value : SortedJointArray)
	{
		XMVECTOR convertedEuler;
		QuaternionToEulerAngles(value->RefQuat, convertedEuler);

		//if (abs(XMVectorGetX(convertedEuler) - XMVectorGetX(value->RefEuler)) > 0.1f ||
		//	abs(XMVectorGetY(convertedEuler) - XMVectorGetY(value->RefEuler)) > 0.1f ||
		//	abs(XMVectorGetZ(convertedEuler) - XMVectorGetZ(value->RefEuler)) > 0.1f)
		//{
		//	int a = 0;
		//}
	}


	// world Position/World Rotation 생성
	for (auto const& value : SortedJointArray)
	{
		if (value->ParentJoint)
		{
			value->WorldPosition = XMVectorAdd(value->ParentJoint->WorldPosition, value->Position);
			value->WorldQuat = XMQuaternionMultiply(value->RefQuat, value->ParentJoint->WorldQuat);
		}
		else
		{
			value->WorldPosition = value->Position;
			value->WorldQuat = value->RefQuat;
		}
	}

	XMVECTOR zeroVector = { 0.0f, 0.0f, 0.0f, 0.0f };
	XMVECTOR identityVector = { 1.0f, 1.0f, 1.0f, 1.0f };

	XMVECTOR vectorY = { 0.0f, 1.0f, 0.0, 0.0f };

	// world transform을 이용한 위치 구하기
	for (auto const& value : SortedJointArray)
	{
		if (value->ParentJoint)
		{
			XMVECTOR loc = value->CalculateWorldPostionByParentJoint();

			XMVECTOR diff = XMVector3Length(value->WorldPosition - loc);

			if (XMVectorGetX(diff) > 0.01f)
			{
				int a = 0;
			}

			if (value->ChildrenJoint.size() > 1)
			{
				XMVECTOR childHeadLocation = zeroVector;

				for (auto const& child : value->ChildrenJoint)
				{
					if (XMVector3Equal(childHeadLocation, zeroVector))
					{
						childHeadLocation = child->CalculateWorldPostionByParentJoint();
					}
					else
					{
						XMVECTOR diff2 = XMVector3Length(childHeadLocation - child->CalculateWorldPostionByParentJoint());
						if (XMVectorGetX(diff2) > 0.01f)
						{
							int a = 0;
						}
					}
				}

			}
		}
	}
	

	////////////////////////////////////////////////////////////////////////////////////
	// AddJointRotationValue 와 AddJointPositionValue 를 통해 얻은 값이 맞는지 확인
	////////////////////////////////////////////////////////////////////////////////////

	for (auto& rawFrame : RawFrames)
	{
		for (auto i = 0;i < rawFrame.FrameInfo.size(); ++i)
		{
			auto& frameInfo = rawFrame.FrameInfo[i];

			if (!frameInfo.Initialized)
				continue;

			const auto& refPoseInfo = SortedJointArray[i];

			// Position(world), WorldQuat data를 받음.
			// 계산된 LocalQuat와 부터 Parent의 위치에 ParentJoint와 현재 Joint의 거리를 Y축 거리로 보고 
			// 현재 Joint의 위치를 계산해서 비교해 본다.

			auto* joint = SortedJointArray[i];
			auto* parentJoint = joint->ParentJoint;

			// 아래의 두 값이 data로 부터 초기화된 값
			// frameInfo.WorldQuat
			// frameInfo.Position
			
			if (parentJoint == nullptr)
			{
				
			}
			else
			{
				auto& parentFrameInfo = rawFrame.FrameInfo[parentJoint->JointIndex];

				if (parentFrameInfo.Initialized)
				{
					const XMVECTOR zeroVector = { 0.0f, 0.0f, 0.0f, 0.0f };
					const XMVECTOR identityVector = { 1.0f, 1.0f, 1.0f, 1.0f };
					const XMVECTOR vectorY = { 0.0f, 1.0f, 0.0, 0.0f };

					XMVECTOR boneDirection = XMVector3Rotate(vectorY, frameInfo.WorldQuat);

					XMVECTOR boneLength = XMVector3Length(parentFrameInfo.Position - frameInfo.Position);
					XMVECTOR loc = parentFrameInfo.Position + boneDirection*boneLength;

					XMVECTOR diff = XMVector3Length(frameInfo.Position - loc);
					if (XMVectorGetX(diff) > 0.01f)
					{
						int a = 0;
					}
				}
			}
		}
	}
}

void CBVH::SetJointCount(int inCount)
{
	if (inCount > 0)
	{
		JointCount = inCount;
		JointNames.resize(inCount);
		JointParentNames.resize(inCount);
		JointBoneDirections.resize(inCount);
	}
}

void CBVH::SetJointConfigure(int inJointIndex, const std::string & inJointName, const std::string & inJointParentName, EKinectJointBoneDirection inBoneDirection)
{
	if (inJointIndex < JointCount)
	{
		JointNames[inJointIndex] = inJointName;
		JointParentNames[inJointIndex] = inJointParentName;
		JointBoneDirections[inJointIndex] = inBoneDirection;
	}
}

void CBVH::SetKinectBoneConfiguration()
{
	SetJointCount(JointType_Count);

	// https://social.msdn.microsoft.com/Forums/en-US/f2e6a544-705c-43ed-a0e1-731ad907b776/meaning-of-rotation-data-of-k4w-v2

	SetJointConfigure(JointType_SpineBase, "SpineBase", "", EKinectJointBoneDirection_Y);
	SetJointConfigure(JointType_SpineMid, "SpineMid", "SpineBase", EKinectJointBoneDirection_Y);
	SetJointConfigure(JointType_Neck, "Neck", "SpineShoulder", EKinectJointBoneDirection_Y);
	SetJointConfigure(JointType_Head, "Head", "Neck", EKinectJointBoneDirection_Y);
	SetJointConfigure(JointType_ShoulderLeft, "ShoulderLeft", "SpineShoulder", EKinectJointBoneDirection_X);
	SetJointConfigure(JointType_ElbowLeft, "ElbowLeft", "ShoulderLeft", EKinectJointBoneDirection_X);
	SetJointConfigure(JointType_WristLeft, "WristLeft", "ElbowLeft", EKinectJointBoneDirection_Y);
	SetJointConfigure(JointType_HandLeft, "HandLeft", "WristLeft", EKinectJointBoneDirection_Y);
	SetJointConfigure(JointType_ShoulderRight, "ShoulderRight", "SpineShoulder", EKinectJointBoneDirection_NX);
	SetJointConfigure(JointType_ElbowRight, "ElbowRight", "ShoulderRight", EKinectJointBoneDirection_NX);
	SetJointConfigure(JointType_WristRight, "WristRight", "ElbowRight", EKinectJointBoneDirection_Y);
	SetJointConfigure(JointType_HandRight, "HandRight", "WristRight", EKinectJointBoneDirection_Y);
	SetJointConfigure(JointType_HipLeft, "HipLeft", "SpineBase", EKinectJointBoneDirection_X);
	SetJointConfigure(JointType_KneeLeft, "KneeLeft", "HipLeft", EKinectJointBoneDirection_X);
	SetJointConfigure(JointType_AnkleLeft, "AnkleLeft", "KneeLeft", EKinectJointBoneDirection_Y);
	SetJointConfigure(JointType_FootLeft, "FootLeft", "AnkleLeft", EKinectJointBoneDirection_Y);
	SetJointConfigure(JointType_HipRight, "HipRight", "SpineBase", EKinectJointBoneDirection_NX);
	SetJointConfigure(JointType_KneeRight, "KneeRight", "HipRight", EKinectJointBoneDirection_NX);
	SetJointConfigure(JointType_AnkleRight, "AnkleRight", "KneeRight", EKinectJointBoneDirection_Y);
	SetJointConfigure(JointType_FootRight, "FootRight", "AnkleRight", EKinectJointBoneDirection_Y);
	SetJointConfigure(JointType_SpineShoulder, "SpineShoulder", "SpineMid", EKinectJointBoneDirection_Y);
	SetJointConfigure(JointType_HandTipLeft, "HandTipLeft", "HandLeft", EKinectJointBoneDirection_Y);
	SetJointConfigure(JointType_ThumbLeft, "ThumbLeft", "HandLeft", EKinectJointBoneDirection_Y);
	SetJointConfigure(JointType_HandTipRight, "HandTipRight", "HandRight", EKinectJointBoneDirection_Y);
	SetJointConfigure(JointType_ThumbRight, "ThumbRight", "HandRight", EKinectJointBoneDirection_Y);

	ResetJointParentIndex();
}

void CBVH::AddJointOffsetValue(int inJointIndex, float inX, float inY, float inZ)
{
	if (inJointIndex < (int)IndexConvertTable.size())
	{
		int sortedIndex = IndexConvertTable[inJointIndex];

		if (sortedIndex < (int)SortedJointArray.size())
		{
			SortedJointArray[inJointIndex]->Position = { inX, inY, inZ };
		}
	}
}

void CBVH::ExportFile(const std::string & inFileName)
{
	GenerateLocalRotation();

	DataValidationTest();

	GenerateEvenSpacedFrameData();

	if (RootJoint)
	{ 
		std::string content;

		RootJoint->ExportHIERARCHY(content);

		content.append("MOTION\n");

		content.append("Frames: ");
		content.append(std::to_string(Frames.size()));
		content.append("\n");

		content.append("Frame Time: ");
		content.append(std::to_string(1.0f/(float)ExportFrameRate));
		content.append("\n");

		for (auto& value : Frames)
		{
			value.ExportMOTION(content, false);
		}

		std::ofstream myfile;
		myfile.open(inFileName.c_str());
		myfile << content;

		myfile.close();
	}


}

FBVHJoint::FBVHJoint() : ParentJoint(nullptr), KinectJointType(JointType_Count)
{
}

FBVHJoint::FBVHJoint(FBVHJoint* inParentJoint) : ParentJoint(inParentJoint)
{
}

FBVHJoint::~FBVHJoint()
{
	ParentJoint = nullptr;

	for (auto& value : ChildrenJoint)
	{
		delete value;
	}
}

void FBVHJoint::GatherJoints(std::vector<FBVHJoint*>& outJoints)
{
	outJoints.push_back(this);
	for (auto value : ChildrenJoint)
	{
		if (value->ChildrenJoint.size() > 0)
			value->GatherJoints(outJoints);
	}
}

char* Tabs[] = 
{
	"",
	"\t",
	"\t\t",
	"\t\t\t",
	"\t\t\t\t",
	"\t\t\t\t\t",
	"\t\t\t\t\t\t",
	"\t\t\t\t\t\t\t",
	"\t\t\t\t\t\t\t\t",
	"\t\t\t\t\t\t\t\t\t",
	"\t\t\t\t\t\t\t\t\t\t",
	"\t\t\t\t\t\t\t\t\t\t\t",
};


void FBVHJoint::ExportHIERARCHY(std::string & outData, int inDepth)
{
	if (ParentJoint == NULL)
	{
		outData.append("HIERARCHY\n");
		outData.append("ROOT ");
	}
	else if (ChildrenJoint.size() > 0)
	{
		outData.append(Tabs[inDepth]);
		outData.append("JOINT ");
	}
	else
	{
		outData.append(Tabs[inDepth]);
		outData.append("End ");
	}

	outData.append(JointName);outData.append("\n");

	outData.append(Tabs[inDepth]); outData.append("{\n");

	outData.append(Tabs[inDepth + 1]); outData.append("OFFSET");

	//float offsetX = 0.0f, offsetY = 0.0f, offsetZ = 0.0f;

	//if (BoneDirection == EKinectJointBoneDirection_X)
	//{
	//	offsetX = 50.0f;
	//}
	//else if (BoneDirection == EKinectJointBoneDirection_Y)
	//{
	//	offsetY = 50.0f;
	//}
	//else if (BoneDirection == EKinectJointBoneDirection_Z)
	//{
	//	offsetZ = 50.0f;
	//}
	//else if (BoneDirection == EKinectJointBoneDirection_NX)
	//{
	//	offsetX = -50.0f;
	//}
	//else if (BoneDirection == EKinectJointBoneDirection_NY)
	//{
	//	offsetY = -50.0f;
	//}
	//else if (BoneDirection == EKinectJointBoneDirection_NZ)
	//{
	//	offsetZ = -50.0f;
	//}

	outData.append(" ");
	outData.append(std::to_string(XMVectorGetX(Position)));
	outData.append(" ");
	outData.append(std::to_string(XMVectorGetY(Position)));
	outData.append(" ");
	outData.append(std::to_string(XMVectorGetZ(Position)));
	outData.append("\n");

	if (ParentJoint == NULL)
	{
		outData.append(Tabs[inDepth + 1]);
		outData.append("CHANNELS 6 Xposition Yposition Zposition Xrotation Yrotation Zrotation\n");
	}
	else if (ChildrenJoint.size() > 0)
	{
		outData.append(Tabs[inDepth + 1]);
		outData.append("CHANNELS 3 Xrotation Yrotation Zrotation\n");
	}

	for (auto value : ChildrenJoint)
	{
		value->ExportHIERARCHY(outData, inDepth + 1);
	}

	outData.append(Tabs[inDepth]); outData.append("}\n");
}

XMVECTOR FBVHJoint::CalculateWorldPostionByParentJoint()
{
	// Parent의 위치를 기준으로 해서, 
	// 현재 Joint 의 Bone길이를 기준으로 현재 Joint의 위치(world 좌표계)를 구함
	// bone의 방향을 Y방향을 계산

	const XMVECTOR zeroVector = { 0.0f, 0.0f, 0.0f, 0.0f };
	const XMVECTOR identityVector = { 1.0f, 1.0f, 1.0f, 1.0f };
	const XMVECTOR vectorY = { 0.0f, 1.0f, 0.0, 0.0f };

	if (ParentJoint)
	{
		XMVECTOR boneDirection = XMVector3Rotate(vectorY, ParentJoint->WorldQuat);

		XMVECTOR boneLength = XMVector3Length(Position);
		XMVECTOR loc = ParentJoint->WorldPosition + boneDirection*boneLength;

		return loc;
	}

	return zeroVector;
}

void FBVHFrame::ExportMOTION(std::string & outData, bool bQuaternion)
{
	const float convertRad2Deg = 180.0f / XM_PI;
	outData += "0.0 0.0 0.0";

	int index = 0;
	for (auto value : FrameInfo)
	{
		if (bQuaternion)
		{
			outData += " " + std::to_string(value.DevQuat.m128_f32[0]);
			outData += " " + std::to_string(value.DevQuat.m128_f32[1]);
			outData += " " + std::to_string(value.DevQuat.m128_f32[2]);
			outData += " " + std::to_string(value.DevQuat.m128_f32[3]);

			//outData += " " + std::to_string(value.LocalQuat.m128_f32[0]);
			//outData += " " + std::to_string(value.LocalQuat.m128_f32[1]);
			//outData += " " + std::to_string(value.LocalQuat.m128_f32[2]);
			//outData += " " + std::to_string(value.LocalQuat.m128_f32[3]);
		}
		else
		{
			outData += " " + std::to_string(XMVectorGetX(value.Rotation)*convertRad2Deg);
			outData += " " + std::to_string(XMVectorGetY(value.Rotation)*convertRad2Deg);
			outData += " " + std::to_string(XMVectorGetZ(value.Rotation)*convertRad2Deg);

			if (std::isnan(XMVectorGetX(value.Rotation)) ||
				std::isnan(XMVectorGetY(value.Rotation)) ||
				std::isnan(XMVectorGetZ(value.Rotation)))
			{
				int a = 0;
			}
			//outData += " " + std::to_string(std::get<0>(value.Rotation));
			//outData += " " + std::to_string(std::get<1>(value.Rotation));
			//outData += " " + std::to_string(std::get<2>(value.Rotation));
			//outData += " " + std::to_string(0.0f);
			//outData += " " + std::to_string(0.0f);
			//outData += " " + std::to_string(0.0f);

			//if (index == 4)
			//{
			//	static float angle = 0.0f;

			//	//outData += " " + std::to_string(angle / 180.0f*XM_PI);
			//	outData += " " + std::to_string(angle);
			//	outData += " " + std::to_string(0.0f);
			//	outData += " " + std::to_string(0.0f);

			//	angle += 0.5f;
			//}
			//else
			//{
			//	outData += " " + std::to_string(0.0f);
			//	outData += " " + std::to_string(0.0f);
			//	outData += " " + std::to_string(0.0f);
			//}
		}

		index++;
	}
	outData.append("\n");
}
