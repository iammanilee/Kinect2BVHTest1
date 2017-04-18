#pragma once

#include <vector>
#include <tuple>
#include <unordered_map>
#include <string>
#include <Kinect.h>
#include <DirectXMath.h>

// bvh 는 bone 길이 변화를 허용하지 않음.

using namespace DirectX;


// https://social.msdn.microsoft.com/Forums/en-US/f2e6a544-705c-43ed-a0e1-731ad907b776/meaning-of-rotation-data-of-k4w-v2
enum EKinectJointBoneDirection
{
	EKinectJointBoneDirection_X,
	EKinectJointBoneDirection_Y,
	EKinectJointBoneDirection_Z,
	EKinectJointBoneDirection_NX,
	EKinectJointBoneDirection_NY,
	EKinectJointBoneDirection_NZ,
};

inline void QuaternionToEulerAngles(const XMVECTOR& inQuat, XMVECTOR& outEulerianAngles)
{
	float q1x = XMVectorGetX(inQuat);
	float q1y = XMVectorGetY(inQuat);
	float q1z = XMVectorGetZ(inQuat);
	float q1w = XMVectorGetW(inQuat);

	float heading = 0.0f, attitude = 0.0f, bank = 0.0f;

	float test = q1x*q1y + q1z*q1w;
	if (test > 0.499) { // singularity at north pole
		heading = 2 * atan2(q1x, q1w);
		attitude = XM_PIDIV2;
		bank = 0;
	}
	if (test < -0.499) { // singularity at south pole
		heading = -2 * atan2(q1x, q1w);
		attitude = -XM_PIDIV2;
		bank = 0;
	}
	else
	{
		float sqx = q1x*q1x;
		float sqy = q1y*q1y;
		float sqz = q1z*q1z;
		heading = atan2(2 * q1y*q1w - 2 * q1x*q1z, 1 - 2 * sqy - 2 * sqz);
		attitude = asin(2 * test);
		bank = atan2(2 * q1x*q1w - 2 * q1y*q1z, 1 - 2 * sqx - 2 * sqz);
	}

	outEulerianAngles = XMVectorSetX(outEulerianAngles, heading);
	outEulerianAngles = XMVectorSetY(outEulerianAngles, attitude);
	outEulerianAngles = XMVectorSetZ(outEulerianAngles, bank);

	//std::get<0>(outEulerianAngles) = heading;			// pitch
	//std::get<1>(outEulerianAngles) = attitude;			// yaw
	//std::get<2>(outEulerianAngles) = bank;				// roll

	//std::get<0>(outEulerianAngles) = heading;			// pitch
	//std::get<2>(outEulerianAngles) = attitude;			// yaw
	//std::get<1>(outEulerianAngles) = bank;				// roll

	//std::get<1>(outEulerianAngles) = heading;			// pitch
	//std::get<0>(outEulerianAngles) = attitude;			// yaw
	//std::get<2>(outEulerianAngles) = bank;				// roll

	//std::get<1>(outEulerianAngles) = heading;			// pitch
	//std::get<2>(outEulerianAngles) = attitude;			// yaw
	//std::get<0>(outEulerianAngles) = bank;				// roll

	//std::get<2>(outEulerianAngles) = heading;			// pitch
	//std::get<1>(outEulerianAngles) = attitude;			// yaw
	//std::get<0>(outEulerianAngles) = bank;				// roll

	//std::get<2>(outEulerianAngles) = heading;			// pitch
	//std::get<0>(outEulerianAngles) = attitude;			// yaw
	//std::get<1>(outEulerianAngles) = bank;				// roll

}

inline XMVECTOR Vector4ToXMVECTOR(const Vector4& inValue)
{
	XMVECTOR tmp;
	tmp.m128_f32[0] = inValue.x;
	tmp.m128_f32[1] = inValue.y;
	tmp.m128_f32[2] = inValue.z;
	tmp.m128_f32[3] = inValue.w;
	return tmp;
}


struct FBVHJointTransform
{
	XMVECTOR Position;		// Position
	XMVECTOR Rotation;		// Relative Rotation to parent

	XMVECTOR LocalQuat;			// quaternion
	XMVECTOR WorldQuat;			// quaternion
	XMVECTOR DevQuat;			// devation from refPose

	bool Initialized;

	FBVHJointTransform() { ZeroMemory(this, sizeof(FBVHJointTransform)); }
};

struct FBVHJoint
{
	int JointIndex;
	FBVHJoint* ParentJoint;
	std::vector<FBVHJoint*> ChildrenJoint;
	std::string JointName;
	EKinectJointBoneDirection BoneDirection;

	JointType KinectJointType;

	XMVECTOR Position;	// relative position | World Coordinate

	XMVECTOR RefQuat;							// ref quaternion
	XMVECTOR InvRefQuat;						// inverse of ref quaternion

	XMVECTOR WorldPosition;	// position | World Coordinate
	XMVECTOR WorldQuat;
	XMVECTOR LocalPosition;	// Local position to Parent Joint | Local Coordinate


	FBVHJoint();
	FBVHJoint(FBVHJoint* inParentJoint);

	~FBVHJoint();
	

	void GatherJoints(std::vector<FBVHJoint*>& outJoints);
	void ExportHIERARCHY(std::string& outData, int inDepth = 0);

	XMVECTOR CalculateWorldPostionByParentJoint();
};

struct FRawBVHFrame
{
	DWORD ElapseTime;				// milliseconds
	std::vector<FBVHJointTransform> FrameInfo;
};

struct FBVHFrame
{
	DWORD ElapseTime;				// milliseconds
	std::vector<FBVHJointTransform> FrameInfo;
	void ExportMOTION(std::string& outData, bool bQuaternion);
};

class CBVH
{
	int NumberOfFrames;
	int NumberOfFramesInSecond;

	int JointCount;
	std::vector<std::string> JointNames;
	std::vector<std::string> JointParentNames;
	std::vector<EKinectJointBoneDirection> JointBoneDirections;

	std::unordered_map<std::string, JointType> NameJointTypeMap;

	FBVHJoint* RootJoint;
	std::vector<FBVHJoint*> SortedJointArray;		// Parent-Child 관계가 고려된 순서 : Parent의 index는 child보다 항상 작다.
	std::vector<int> IndexConvertTable;				// SetJointName() 로 입력된 TableIndex를 JointArray의 Index로 변환하는 Table

	std::vector<FRawBVHFrame> RawFrames;		// 직접 기록한 Frame 정보
	std::vector<FBVHFrame> Frames;				// Raw Data로 부터 일정한 간격의 Frame 값으로 만들어낸 값들

	// 
	const DWORD INVALID_ELAPSE_TIME = 0xffffffff;
	DWORD CurrentElapseTime;					// milliseconds

	const int ExportFrameRate = 30;

	FRawBVHFrame* CurrentRawBVHFrame;

	void GenerateLocalRotation();

	void GenerateEvenSpacedFrameData();

	void MakeNameJointTypeMap();
	JointType GetJointType(const std::string& inJointName);

public:
	CBVH();
	~CBVH();

	// Set Number of Joint
	void SetJointCount(int inCount);

	// Define Joint Configuration 
	void SetJointConfigure(int inJointIndex, const std::string& inJointName, const std::string& inJointParentName, EKinectJointBoneDirection inBoneDirection);

	// Set Kinect Configuration
	void SetKinectBoneConfiguration();

	// Define Joint Value 
	void AddJointOffsetValue(int inJointIndex, float inX, float inY, float inZ);

	void ResetJointParentIndex();

	void Begin(DWORD inMilliSeconds);

	void AddJointRotationValue(JointType inKinectJointType, const XMVECTOR& inQuat);
	void AddJointPositionValue(JointType inKinectJointType, const XMVECTOR& inPosition);

	void End();

	void ImportRefPoseByBVHFile(const std::string& inFileName);

	void DataValidationTest();

	void ExportFile(const std::string& inFileName);

//#ifdef __Kinect_h__
//	static void toEulerianAngle(const Vector4& q, float& roll, float& pitch, float& yaw)
//	{
//		double ysqr = q.y * q.y;
//
//		// roll (x-axis rotation)
//		double t0 = +2.0 * (q.w * q.x + q.y * q.z);
//		double t1 = +1.0 - 2.0 * (q.x * q.x + ysqr);
//		roll = std::atan2(t0, t1);
//
//		// pitch (y-axis rotation)
//		double t2 = +2.0 * (q.w * q.y - q.z * q.x);
//		t2 = t2 > 1.0 ? 1.0 : t2;
//		t2 = t2 < -1.0 ? -1.0 : t2;
//		pitch = std::asin(t2);
//
//		// yaw (z-axis rotation)
//		double t3 = +2.0 * (q.w * q.z + q.x * q.y);
//		double t4 = +1.0 - 2.0 * (ysqr + q.z * q.z);
//		yaw = std::atan2(t3, t4);
//	}
//
//	static Vector4 toQuaternion(float pitch, float roll, float yaw)
//	{
//		Vector4 q;
//		float t0 = std::cos(yaw * 0.5);
//		float t1 = std::sin(yaw * 0.5);
//		float t2 = std::cos(roll * 0.5);
//		float t3 = std::sin(roll * 0.5);
//		float t4 = std::cos(pitch * 0.5);
//		float t5 = std::sin(pitch * 0.5);
//
//		q.w = t0 * t2 * t4 + t1 * t3 * t5;
//		q.x = t0 * t3 * t4 - t1 * t2 * t5;
//		q.y = t0 * t2 * t5 + t1 * t3 * t4;
//		q.z = t1 * t2 * t4 - t0 * t3 * t5;
//		return q;
//	}
//#endif
};

