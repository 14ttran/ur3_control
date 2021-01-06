/*
@file motion_planning.h Class and function declarations for motion planning utilities, ex 3D transformations, units in mm and deg

@language C++14

@author 14ttran
*/

#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "stl_reader.h"
// #include "Eigen/src/Core/Matrix.h"
// #include "Eigen/src/Geometry/AngleAxis.h"


constexpr float  PI_F = 3.14159265358979f; // for speeeeeeeeed
#define DegreesToRadians(fAngleDegrees) ((fAngleDegrees) * PI_F / 180.0)
#define RadiansToDegrees(fAngleRadians) ((fAngleRadians) * 180.0 / PI_F)

namespace math
{
float square(const float& fValue);
float root(const float& fValue);
float dot(const std::vector<float>& lstfVec1, const std::vector<float>& lstfVec2);
std::vector<float> cross(const std::vector<float>& lstfVec1, const std::vector<float>& lstfVec2);
float normalize(const std::vector<float>& lstfVec);
std::vector<float> subtract(const std::vector<float>& lstfVec1, const std::vector<float>& lstfVec2);
std::vector<float> scale(const std::vector<float>& lstfVec, const float& fScale);
}

class CAngleAxisPose : public Eigen::Matrix<float, 6, 1>
{
private:
    Eigen::AngleAxisf m_cAngleAxis;
    Eigen::AngleAxisf AngleAxisDecomposition(float fRx, float fRy, float fRz);
    std::vector<float> AngleAxisComposition(float fR, float fUx, float fUy, float fUz);

public:
    CAngleAxisPose(float fX, float fY, float fZ, float fRx, float fRy, float fRz);
    // ~CAngleAxisPose();

    // Eigen inheritance
    CAngleAxisPose();
    // This constructor allows you to construct MyVectorType from Eigen expressions
    template<typename OtherDerived>
    CAngleAxisPose(const Eigen::MatrixBase<OtherDerived>& other);
    // This method allows you to assign Eigen expressions to MyVectorType
    template<typename OtherDerived>
    CAngleAxisPose& operator=(const Eigen::MatrixBase <OtherDerived>& other)
    {
        this->Eigen::Matrix<float, 6, 1>::operator=(other);
        return *this;
    }
    CAngleAxisPose& operator=(const std::vector<float>& lstfVector)
    {
        CAngleAxisPose clstfTemp {lstfVector[0], lstfVector[1], lstfVector[2], lstfVector[3], lstfVector[4], lstfVector[5]};
        *this = clstfTemp;
        return *this;
    }
    CAngleAxisPose& FromFrame(const Eigen::Matrix4f& lstlstfFrame);

    CAngleAxisPose TransformToCoordinateSystem(CAngleAxisPose lstlstfNewCS);
    Eigen::Matrix4f Frame();
    std::vector<float> Vector();
    std::vector<float> CartesianComponent();
    std::vector<float> RotationComponent();
    float RotationZ();
    CAngleAxisPose PostMultiply(CAngleAxisPose clstfPose, bool bInverseFlag=false);
    CAngleAxisPose PreMultiply(CAngleAxisPose clstfPose, bool bInverseFlag=false);
    CAngleAxisPose GetApproachPose(std::vector<float> lstfApproachDirection);
};

std::vector<float> MakeBoundaryPlane(const std::vector<float>& lstfNormal, const std::vector<float>& lstfPointOnPlane);
Eigen::Vector4f MakeBoundaryPlane(const Eigen::Vector3f& lstfNormal, const Eigen::Vector3f& lstfPointOnPlane);

bool CheckBoundaryPlane(const std::vector<float>& lstfPlane, const std::vector<float>& lstfPoint);
bool CheckBoundaryPlane(const Eigen::Vector3f& lstfNormal, const Eigen::Vector3f& lstfPointOnPlane, const Eigen::Vector3f& lstfPoint);

std::vector<float> GetPointProjection(const std::vector<float>& lstfNormal, const std::vector<float>& lstfVertex, const std::vector<float>& lstfPoint);
Eigen::Vector3f GetPointProjection(const Eigen::Vector3f& lstfNormal, const Eigen::Vector3f& lstfVertex, const Eigen::Vector3f& lstfPoint);

bool CheckInsideFacet(const std::vector<float>& lstfPoint, const std::vector<float>& lstfVertex1, const std::vector<float>& lstfVertex2, const std::vector<float>& lstfVertex3, const float& fArea);
bool CheckInsideFacet(const Eigen::Vector3f& lstfPoint, const Eigen::Vector3f& lstfVertex1, const Eigen::Vector3f& lstfVertex2, const Eigen::Vector3f& lstfVertex3, const float& fArea);

bool CheckFacetCollision(const std::vector<std::vector<float>>& lstlstfFacet, const std::vector<float>& lstfFacetInfo, const std::vector<float>& lstfPlane, const std::vector<float>& lstfPoint, const bool& bForceCheck=false);
bool CheckFacetCollision(const std::vector<Eigen::Vector3f>& lstlstfFacet, const std::vector<float>& lstfFacetInfo, const Eigen::Vector3f& lstfPoint, const bool& bForceCheck=false);

bool DetectObjectCollision(const std::vector<std::vector<float>>& lstlstfPC, const std::vector<std::vector<std::vector<float>>>& lstlstfSTLFacets, const std::vector<std::vector<float>>& lstlstfSTLFacetInfo, const std::vector<std::vector<float>>& lstlstfSTLPlanes);
bool DetectObjectCollision(const std::vector<Eigen::Vector3f>& lstlstfPC, const std::vector<std::vector<Eigen::Vector3f>>& lstlstfSTLFacets, const std::vector<std::vector<float>>& lstlstfSTLFacetInfo);

Eigen::Vector3f RayPlaneIntersection(const Eigen::Vector3f& eigPlaneNormal, const Eigen::Vector3f& eigPointOnPlane, const Eigen::Vector3f& eigRayDirection, const Eigen::Vector3f& eigPointOnRay, float fEpsilon=1e-06);
std::vector<std::vector<Eigen::Vector3f>> LoadStlMesh(const std::string& strStlPath);
std::vector<std::vector<float>> GetMeshData(const std::vector<std::vector<Eigen::Vector3f>>& lstlstfSTLFacets);

std::vector<std::vector<Eigen::Vector3f>> ReverseForwardKinematics(CAngleAxisPose clstfRobotPose_w, const CAngleAxisPose& clstfTcp_f, const std::vector<Eigen::Vector3f>& lsteigPointsOfInterest_f, const std::vector<float>& lstfJointAngles);
Eigen::Vector3f RodriguesRotation(const Eigen::Vector3f& eigObject, const Eigen::Vector3f& eigRotation);

std::vector<Eigen::Vector3f> GeneratePointCloudCylinder(const Eigen::Vector3f& eigCylinderCenter, const Eigen::Vector3f& eigDirection, const float& fRadius, const float& fLength, const int& nSlices, const int& nPointsOnSlice);

Eigen::Vector3f RotationBetweenVectors(const Eigen::Vector3f& eigVec1, const Eigen::Vector3f& eigVec2);
Eigen::Vector3f RotationMatrixToVector(const Eigen::Matrix3f& eigRotation);

std::vector<std::vector<float>> LoadPath(const std::string& strPath, const int& nCoordinateLength=3);
std::vector<CAngleAxisPose> Generate6DofPath(const std::vector<std::vector<float>>& lstlstf2DPath, const CAngleAxisPose& lstfPathOffset={0,0,0,0,0,0}, const CAngleAxisPose& lstfTilt={0,0,0,0,0,0});

std::vector<CAngleAxisPose> Generate6DofTab(const std::vector<std::vector<float>>& lstlstf2DPath, const CAngleAxisPose& lstfPathOffset={0,0,0,0,0,0}, const CAngleAxisPose& lstfTilt={0,0,0,0,-30,0});
