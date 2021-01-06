/*
@file motion_planning.cpp Class and function definitions for motion planning utilities, ex 3D transformations, units in mm and deg

@language C++14

@author 14ttran

Thomas Tran motion planning utilities, ex 3D transformations, and mesh loading and collision detection units in mm and deg
Copyright (C) 2021 Thomas Tran

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>
*/

#include "motion_planning.h"

using namespace std;

CAngleAxisPose::CAngleAxisPose()
: Eigen::Matrix<float, 6, 1> {}
{
    /**
     * @brief Empty construtor to create the interal members for loading from frames
     */
    
}

CAngleAxisPose::CAngleAxisPose(float fX, float fY, float fZ, float fRx, float fRy, float fRz)
: Eigen::Matrix<float, 6, 1> {}
{
    /**
     * @brief Constructor for creating from angle-axis values
     * @param fX X coordinate in mm
     * @param fY Y coordinate in mm
     * @param fZ Z coordinate in mm
     * @param fRx X component of rotation vector in deg
     * @param fRy Y component of rotation vector in deg
     * @param fRz Z component of rotation vector in deg
     */
    (*this)(0) = fX;
    (*this)(1) = fY;
    (*this)(2) = fZ;
    (*this)(3) = fRx;
    (*this)(4) = fRy;
    (*this)(5) = fRz;

    m_cAngleAxis = AngleAxisDecomposition(DegreesToRadians((*this)(3)), DegreesToRadians((*this)(4)), DegreesToRadians((*this)(5)));
}

Eigen::AngleAxisf CAngleAxisPose::AngleAxisDecomposition(float fRx, float fRy, float fRz)
{
    /**
     * @brief Decomposes an angle-axis rotation vector into it's unit vector and angle of rotation in radians
     * @param fRx X component of rotation vector in rad
     * @param fRy Y component of rotation vector in rad
     * @param fRz Z component of rotation vector in rad
     * @return AngleAxisf object with rotation angle in rad and axis unit vector
     */
    float fR = math::root( math::square(fRx) + math::square(fRy) + math::square(fRz) );
    Eigen::Vector3f lstfUnitVector {};
    if (fR == 0.0)
    {
        lstfUnitVector[0] = fRx;
        lstfUnitVector[1] = fRy;
        lstfUnitVector[2] = fRz;
    }
    else 
    {
        lstfUnitVector[0] = fRx / fR;
        lstfUnitVector[1] = fRy / fR;
        lstfUnitVector[2] = fRz / fR;
    }

    return Eigen::AngleAxisf {fR, lstfUnitVector};
}

vector<float> CAngleAxisPose::AngleAxisComposition(float fR, float fUx, float fUy, float fUz)
{
    /**
     * @brief Composes and angle-axis rotation vector from a unit vector and rotation angle in rad
     * @param fR Rotation angle in radians
     * @param fUx X component of axis unit vector
     * @param fUy Y component of axis unit vector
     * @param fUz Z component of axis unit vector
     * @return Axis angle rotation vector in radians
     */
    float fRx = fR * fUx;
    float fRy = fR * fUy;
    float fRz = fR * fUz;

    vector<float> lstfRotVec {fRx, fRy, fRz};
    return lstfRotVec;
}

Eigen::Matrix4f CAngleAxisPose::Frame()
{
    /**
     * @brief Constructs a 4x4 matrix representing a frame 
     * @return Matrix4f object representing the frame
     */
    Eigen::MatrixXf lstlstTemp {m_cAngleAxis.toRotationMatrix()};
    lstlstTemp.conservativeResize(4, 4);
    lstlstTemp(0,3) = (*this)(0);
    lstlstTemp(1,3) = (*this)(1);
    lstlstTemp(2,3) = (*this)(2);
    lstlstTemp(3,0) = 0;
    lstlstTemp(3,1) = 0;
    lstlstTemp(3,2) = 0;
    lstlstTemp(3,3) = 1;

    Eigen::Matrix4f lstlstFrame = lstlstTemp;
    return lstlstFrame;
    
}

CAngleAxisPose& CAngleAxisPose::FromFrame(const Eigen::Matrix4f& lstlstfFrame)
{
    /**
     * @brief Conversion of frame to angle-axis representation
     * @param lstlstfFrame Matrix4f object representing frame
     * @return Reference to an CAngleAxisPose object
     */
    (*this)(0) = lstlstfFrame(0,3);
    (*this)(1) = lstlstfFrame(1,3);
    (*this)(2) = lstlstfFrame(2,3);
    Eigen::MatrixXf lstlstTemp {lstlstfFrame};
    lstlstTemp.conservativeResize(3,3);
    Eigen::Matrix3f lstlstRotMat {lstlstTemp};

    m_cAngleAxis = m_cAngleAxis.fromRotationMatrix(lstlstRotMat);
    Eigen::Vector3f lstfUnitVec = m_cAngleAxis.axis();

    vector<float> lstfRotVec = AngleAxisComposition(m_cAngleAxis.angle(), lstfUnitVec(0), lstfUnitVec(1), lstfUnitVec(2));
    (*this)(3) = RadiansToDegrees(lstfRotVec.at(0));
    (*this)(4) = RadiansToDegrees(lstfRotVec.at(1));
    (*this)(5) = RadiansToDegrees(lstfRotVec.at(2));

    return (*this);
}


CAngleAxisPose CAngleAxisPose::TransformToCoordinateSystem(CAngleAxisPose lstlstfNewCS)
{
    /**
     * @brief Transforms this CAngleAxisPose object into a new coordinate system which is defined in the same coordinate system as the current CAngleAxisPose object
     * @param lstlstfNewCS CAngleAxisPose object representing the new coordinate system location defined in the current coordinate system
     * @return CAngleAxisPose object that's been transformed into the new coordinate system
     */
    Eigen::Matrix4f lstlstfTemp = lstlstfNewCS.Frame().inverse() * (*this).Frame();

    CAngleAxisPose lstlstfNewPose {};
    return lstlstfNewPose.FromFrame(lstlstfTemp);
}

vector<float> CAngleAxisPose::Vector()
{
    /**
     * @brief Generates a 6DOF vector from the object
     * @return 6DOF vector representing the angle-axis pose
     */
    vector<float> lstfValues {(*this)(0), (*this)(1), (*this)(2), (*this)(3), (*this)(4), (*this)(5)};
    return lstfValues;
}

vector<float> CAngleAxisPose::CartesianComponent()
{
    /**
     * @brief Generates the 3DOF vector of the XYZ components from the object
     * @return 3DOF XYZ vector
     */
    vector<float> lstfValues {(*this)(0), (*this)(1), (*this)(2)};
    return lstfValues;
}

vector<float> CAngleAxisPose::RotationComponent()
{
    /**
     * @brief Generates the 3DOF vector of the rotation vector from the object
     * @return 3DOF R vector
     */

    vector<float> lstfValues {(*this)(3), (*this)(4), (*this)(5)};
    return lstfValues;
}

float CAngleAxisPose::RotationZ()
{
    /**
     * @brief Returns the z components of the rotation matrix, ONLY USED FOR 4DOF coordinates
     * @return Z component of rotation in deg
     */

    return (*this)(5);
}

CAngleAxisPose CAngleAxisPose::PostMultiply(CAngleAxisPose clstfPose, bool bInverseFlag)
{
    /**
     * @brief Post multiplys this pose by the parameter pose
     * @param clstfPose CAngleAxis pose to multiply this by in mm and deg
     */
    Eigen::Matrix4f lstlstfTemp;
    if (bInverseFlag == false)
        lstlstfTemp = (*this).Frame() * clstfPose.Frame();
    else 
        lstlstfTemp = (*this).Frame() * clstfPose.Frame().inverse();

    CAngleAxisPose lstlstfNewPose {};
    return lstlstfNewPose.FromFrame(lstlstfTemp);
}

CAngleAxisPose CAngleAxisPose::PreMultiply(CAngleAxisPose clstfPose, bool bInverseFlag)
{
    /**
     * @brief Pre multiplys this pose by the parameter pose
     * @param clstfPose CAngleAxis pose to multiply this by in mm and deg
     */
    Eigen::Matrix4f lstlstfTemp;
    if (bInverseFlag == false) 
        lstlstfTemp = clstfPose.Frame() * (*this).Frame();
    else
        lstlstfTemp = clstfPose.Frame().inverse() * (*this).Frame();

    CAngleAxisPose lstlstfNewPose {};
    return lstlstfNewPose.FromFrame(lstlstfTemp);
}

CAngleAxisPose CAngleAxisPose::GetApproachPose(vector<float> lstfApproachDirection)
{
    /**
     * @brief Calculates and approach pose from this pose based on the given directional unit vector, that to this pose
     * @param lstfApproachDirection 3 DOF unit vector describing the direction the approach is at from this pose in mm
     * @return New angle axis pose
     */

    CAngleAxisPose clstfApproach {lstfApproachDirection[0], lstfApproachDirection[1], lstfApproachDirection[2], 0,0,0};
    vector<float> lstfRotationVector {(*this).RotationComponent()};
    CAngleAxisPose clstfRotation {0,0,0, lstfRotationVector[0], lstfRotationVector[1], lstfRotationVector[2]};

    clstfApproach = clstfApproach.PreMultiply(clstfRotation);
    clstfApproach[0] += (*this)[0];
    clstfApproach[1] += (*this)[1];
    clstfApproach[2] += (*this)[2];

    return clstfApproach;

}

template<typename OtherDerived>
CAngleAxisPose::CAngleAxisPose(const Eigen::MatrixBase<OtherDerived>& other)
: Eigen::Matrix<float, 6, 1>(other)
{
    // don't actually implement
}


float math::square(const float& fValue)
{
    /**
     * @brief Implementation of square
     * @param fValue Float to square
     * @return Squared value
     */
    return pow(fValue, 2);
}

float math::root(const float& fValue)
{
    /**
     * @brief Implementation of square root
     * @param fValue Float to root
     * @return Rooted value
     */
    return sqrt(fValue);
}

float math::dot(const vector<float>& lstfVec1, const vector<float>& lstfVec2)
{
    /**
     * @brief Compute the dot product of two 3D vectors
     * @param lstfVec1 First vector
     * @param lstfVec2 Second vector
     * @return dot product
     */
    float a = lstfVec1[0] * lstfVec2[0];
    float b = lstfVec1[1] * lstfVec2[1];
    float c = lstfVec1[2] * lstfVec2[2];

    return a + b + c;
}

vector<float> math::cross(const vector<float>& lstfVec1, const vector<float>& lstfVec2)
{
    /**
     * @brief Compute the cross product of two 3D vectors
     * @param lstfVec1 First vector
     * @param lstfVec2 Second vector
     * @return cross product
     */
    float x = (lstfVec1[1] * lstfVec2[2]) - (lstfVec1[2] * lstfVec2[1]);
    float y = (lstfVec1[2] * lstfVec2[0]) - (lstfVec1[0] * lstfVec2[2]);
    float z = (lstfVec1[0] * lstfVec2[1]) - (lstfVec1[1] * lstfVec2[0]);

    return {x,y,z};
}

float math::normalize(const vector<float>& lstfVec)
{
    /**
     * @brief Normalize the input vector
     * @param lstfVec 3D vector
     * @return magnitude
     */
    float sum {0.0};
    for (float fComponent : lstfVec)
    {
        sum += square(fComponent);
    }
    return root(sum);
}

vector<float> math::subtract(const vector<float>& lstfVec1, const vector<float>& lstfVec2)
{
    /**
     * @brief Subtracts vector 2 from vector 1
     * @param lstfVec1 First 3d vector
     * @param lstfVec2 Second 3d vector
     * @return new vector
     */
    float x = lstfVec2[0] - lstfVec1[0];
    float y = lstfVec2[1] - lstfVec1[1];
    float z = lstfVec2[2] - lstfVec1[2];

    return {x,y,z};
}

vector<float> math::scale(const vector<float>& lstfVec, const float& fScale)
{
    /**
     * @brief Scales a vector by the given scalar
     * @param lstfVec 3D vector
     * @param fScale Scalar
     * @return Scaled 3D vector
     */
    vector<float> lstfScaled {};
    for (int i = 0; i < lstfVec.size(); i++)
    {
        lstfScaled.push_back(lstfVec[i] * fScale);
    }
    return lstfScaled;
}

vector<float> MakeBoundaryPlane(const vector<float>& lstfNormal, const vector<float>& lstfPointOnPlane)
{
    /**
     * @brief Constructs a vector with plane coefficients from the inputs
     * @param lstfNormal 3D unit vector describing plane normal
     * @param lstfPointOnPlane 3D point on plane
     * @return Vector of plane coefficients
     */

    float fA = lstfNormal[0];
    float fB = lstfNormal[1];
    float fC = lstfNormal[2];
    float fD = -1 * ((fA*lstfPointOnPlane[0]) + (fB*lstfPointOnPlane[1]) + (fC*lstfPointOnPlane[2]));

    return {fA, fB, fC, fD};
}

Eigen::Vector4f MakeBoundaryPlane(const Eigen::Vector3f& lstfNormal, const Eigen::Vector3f& lstfPointOnPlane)
{
    /**
     * @brief Constructs a vector with plane coefficients from the inputs
     * @param lstfNormal 3D unit vector describing plane normal
     * @param lstfPointOnPlane 3D point on plane
     * @return vector of plane coefficients
     */

    float fD = -1 * lstfNormal.dot(lstfPointOnPlane);

    return {lstfNormal[0], lstfNormal[1], lstfNormal[2], fD};
}

bool CheckBoundaryPlane(const vector<float>& lstfPlane, const vector<float>& lstfPoint)
{
    /**
     * @brief Check if the point is on the opposite side of a boundary plane
     * @param lstfPlane 4 param vector with a,b,c,d coefficients where a,b,c describe the unit normal
     * @param lstfPoint 3d point to check
     * @return true if point is passed or on plane, otherwise false
     */
    float fResult = (lstfPlane[0] * lstfPoint[0]) + (lstfPlane[1] * lstfPoint[1]) + (lstfPlane[2] * lstfPoint[2]) + lstfPlane[3];
    if (fResult <= 0)
        return true;
    else
        return false;
}

bool CheckBoundaryPlane(const Eigen::Vector3f& lstfNormal, const Eigen::Vector3f& lstfPointOnPlane, const Eigen::Vector3f& lstfPoint)
{
    /**
     * @brief Check if the point is on the opposite side of a boundary plane
     * @param lstfNormal 3D unit vector describing plane normal
     * @param lstfPointOnPlane 3D point that exists on the boundary plane
     * @param lstfPoint 3d point to check
     * @return true if point is passed or on plane, otherwise false
     */
    float fD = -1*lstfNormal.dot(lstfPointOnPlane);
    float fResult = lstfNormal.dot(lstfPoint) + fD;

    if (fResult <= 0)
        return true;
    else
        return false;
}

vector<float> GetPointProjection(const vector<float>& lstfNormal, const vector<float>& lstfVertex, const vector<float>& lstfPoint)
{
    /**
     * @brief Calculates the orthogonal projection of a point onto the given plane
     * @param lstfNormal 3D normal unit vector of plane
     * @param lstfVertex 3d vertex of a point on the given plane
     * @param lstfPoint 3d point to project
     * @link https://stackoverflow.com/questions/8942950/how-do-i-find-the-orthogonal-projection-of-a-point-onto-a-plane
     * @return 3d projected point
     */
    vector<float> lstfDiff = math::subtract(lstfPoint, lstfVertex);
    float fDot = math::dot(lstfDiff, lstfNormal);
    vector<float> lstfScaledNormal = math::scale(lstfNormal, fDot);

    return math::subtract(lstfPoint, lstfScaledNormal);
}

Eigen::Vector3f GetPointProjection(const Eigen::Vector3f& lstfNormal, const Eigen::Vector3f& lstfVertex, const Eigen::Vector3f& lstfPoint)
{
    /**
     * @brief Calculates the orthogonal projection of a point onto the given plane
     * @param lstfNormal 3D normal unit vector of plane
     * @param lstfVertex 3d vertex of a point on the given plane
     * @param lstfPoint 3d point to project
     * @link https://stackoverflow.com/questions/8942950/how-do-i-find-the-orthogonal-projection-of-a-point-onto-a-plane
     * @return 3d projected point
     */
    Eigen::Vector3f lstfDiff = lstfPoint - lstfVertex;
    float fDot = lstfDiff.dot(lstfNormal);
    Eigen::Vector3f lstfScaledNormal = lstfNormal * fDot;

    return (lstfPoint - lstfScaledNormal);
}

bool CheckInsideFacet(const vector<float>& lstfPoint, const vector<float>& lstfVertex1, const vector<float>& lstfVertex2, const vector<float>& lstfVertex3, const float& fArea)
{
    /**
     * @brief Checks if the given point is within the boundaries created by the 3 given vertices using barycentric math
     * @param lstfPoint 3D point to check
     * @param lstfVertex1 First 3D vertex
     * @param lstfVertex2 Second 3D vertex
     * @param lstfVertex3 Third 3D vertex
     * @param fArea Area of triangle created by 3 vertices
     * @link https://math.stackexchange.com/questions/4322/check-whether-a-point-is-within-a-3d-triangle
     * @return true if point is within or on boundaries, otherwise false
     */
    vector<float> lstfVecPA = math::subtract(lstfVertex1, lstfPoint);
    vector<float> lstfVecPB = math::subtract(lstfVertex2, lstfPoint);
    vector<float> lstfVecPC = math::subtract(lstfVertex3, lstfPoint);

    float a = math::normalize(math::cross(lstfVecPB, lstfVecPC)) / (2*fArea);
    float b = math::normalize(math::cross(lstfVecPC, lstfVecPA)) / (2*fArea);
    float g = math::normalize(math::cross(lstfVecPB, lstfVecPA)) / (2*fArea);
    float total = a + b + g;

    if (a >= 0.0 and a <= 1.0 and b >= 0.0 and b <= 1.0 and g >= 0.0 and g <= 1.0 and total <= 1.0)
        return true;
    else
        return false;
}

bool CheckInsideFacet(const Eigen::Vector3f& lstfPoint, const Eigen::Vector3f& lstfVertex1, const Eigen::Vector3f& lstfVertex2, const Eigen::Vector3f& lstfVertex3, const float& fArea)
{
    /**
     * @brief Checks if the given point is within the boundaries created by the 3 given vertices using barycentric math
     * @param lstfPoint 3D point to check
     * @param lstfVertex1 First 3D vertex
     * @param lstfVertex2 Second 3D vertex
     * @param lstfVertex3 Third 3D vertex
     * @param fArea Area of triangle created by 3 vertices
     * @link https://math.stackexchange.com/questions/4322/check-whether-a-point-is-within-a-3d-triangle
     * @return true if point is within or on boundaries, otherwise false
     */
    Eigen::Vector3f lstfVecPA = lstfVertex1 - lstfPoint;
    Eigen::Vector3f lstfVecPB = lstfVertex2 - lstfPoint;
    Eigen::Vector3f lstfVecPC = lstfVertex3 - lstfPoint;

    float a = lstfVecPB.cross(lstfVecPC).norm() / (2*fArea);
    float b = lstfVecPC.cross(lstfVecPA).norm() / (2*fArea);
    float g = lstfVecPB.cross(lstfVecPA).norm() / (2*fArea);
    float total = a + b + g;

    if (a >= 0.0 and a <= 1.0 and b >= 0.0 and b <= 1.0 and g >= 0.0 and g <= 1.0 and total <= 1.0)
        return true;
    else
        return false;
}

bool CheckFacetCollision(const vector<vector<float>>& lstlstfFacet, const vector<float>& lstfFacetInfo, const vector<float>& lstfPlane, const vector<float>& lstfPoint, const bool& bForceCheck)
{
    /**
     * @brief Checks for a facet collision
     * @param lstlstfFacet Vector with facet features, ex {normal, vertex1, vertex2, vertex3}
     * @param lstfFacetInfo Vector with facet info, ex {area, threshold distance}
     * @param lstfPlane Vector with plane coefficient info related to the given facet, ex {a, b, c, d}
     * @param lstfPoint 3D point to check
     * @param bForceCheck **FOR TESTING** forces a threshold distance check, defaults to false
     * @return true if collision predicted, otherwise false
     */
    bool bPassedPlane = CheckBoundaryPlane(lstfPlane, lstfPoint);
    vector<float> lstfProjection = GetPointProjection(lstlstfFacet[0], lstlstfFacet[1], lstfPoint);
    bool bFacetCollision = CheckInsideFacet(lstfProjection, lstlstfFacet[1], lstlstfFacet[2], lstlstfFacet[3], lstfFacetInfo[0]);
    if (bFacetCollision == true or bForceCheck == true)
    {
        vector<float> lstfDiff = math::subtract(lstfPoint, lstfProjection);
        float fDistance = math::normalize(lstfDiff);
        if (fDistance <= lstfFacetInfo[1])
            bFacetCollision = true;
        else
            bFacetCollision = false;
    }

    return (bPassedPlane and bFacetCollision);
}

bool CheckFacetCollision(const vector<Eigen::Vector3f>& lstlstfFacet, const vector<float>& lstfFacetInfo, const Eigen::Vector3f& lstfPoint, const bool& bForceCheck)
{
    /**
     * @brief Checks for a facet collision
     * @param lstlstfFacet Vector with facet features, ex {normal, vertex1, vertex2, vertex3}
     * @param lstfFacetInfo Vector with facet info, ex {area, threshold distance}
     * @param lstfPoint 3D point to check
     * @param bForceCheck **FOR TESTING** forces a threshold distance check, defaults to false
     * @return true if collision predicted, otherwise false
     */
    bool bPassedPlane = CheckBoundaryPlane(lstlstfFacet[0], lstlstfFacet[1], lstfPoint);
    Eigen::Vector3f lstfProjection = GetPointProjection(lstlstfFacet[0], lstlstfFacet[1], lstfPoint);
    bool bFacetCollision = CheckInsideFacet(lstfProjection, lstlstfFacet[1], lstlstfFacet[2], lstlstfFacet[3], lstfFacetInfo[0]);
    if (bFacetCollision == true or bForceCheck == true)
    {
        Eigen::Vector3f lstfDiff = lstfPoint - lstfProjection;
        float fDistance = lstfDiff.norm();
        if (fDistance <= lstfFacetInfo[1])
            bFacetCollision = true;
        else
            bFacetCollision = false;
    }
    return (bPassedPlane and bFacetCollision);
}

bool DetectObjectCollision(const vector<vector<float>>& lstlstfPC, const vector<vector<vector<float>>>& lstlstfSTLFacets, const vector<vector<float>>& lstlstfSTLFacetInfo, const vector<vector<float>>& lstlstfSTLPlanes)
{
    /**
     * @brief Looping function to detect and object collision between an stl defined object and a pointcloud
     * @param lstlstfPC Point cloud of object 1
     * @param lstlstfSTLFacets Facet data of stl object, same length as lstlstfSTLPlanes
     * @param lstlstfSTLFacetInfo Facet info of stl object, same length as lstlstfSTLFacets
     * @param lstlstfSTLPlanes Plane data of stl object, same length as lstlstfSTLFacets
     * @return true if collision detedted, otherwise false
     */
    for (int j = 0; j < lstlstfPC.size(); j++)
    {
        for (int i = 0; i < lstlstfSTLFacets.size(); i++)
        {
            if (CheckFacetCollision(lstlstfSTLFacets[i], lstlstfSTLFacetInfo[i], lstlstfSTLPlanes[i], lstlstfPC[j], false) == true)
                return true;
        }
    }
    return false;
}

bool DetectObjectCollision(const vector<Eigen::Vector3f>& lstlstfPC, const vector<vector<Eigen::Vector3f>>& lstlstfSTLFacets, const vector<vector<float>>& lstlstfSTLFacetInfo)
{
    /**
     * @brief Looping function to detect and object collision between an stl defined object and a pointcloud
     * @param lstlstfPC Point cloud of object 1
     * @param lstlstfSTLFacets Facet data of stl object, same length as lstlstfSTLFacetInfo
     * @param lstlstfSTLFacetInfo Facet info of stl object, same length as lstlstfSTLFacets
     * @return true if collision detedted, otherwise false
     */

    for (int j = 0; j < lstlstfPC.size(); j++)
    {
        for (int i = 0; i < lstlstfSTLFacets.size(); i++)
        {
            if (CheckFacetCollision(lstlstfSTLFacets[i], lstlstfSTLFacetInfo[i], lstlstfPC[j], false) == true)
            {
                return true;
            }
        }
    }
    return false;
}

Eigen::Vector3f RayPlaneIntersection(const Eigen::Vector3f& eigPlaneNormal, const Eigen::Vector3f& eigPointOnPlane, const Eigen::Vector3f& eigRayDirection, const Eigen::Vector3f& eigPointOnRay, float fEpsilon)
{
    /**
     * @brief Calculates the point where a ray and plane intersect
     * @param eigPlaneNormal 3 DOF unit vector defining the plane normal
     * @param eigPointOnPlane 3 DOF point on the plane
     * @param eigRayDirection 3 DOF unit vector defining the ray direction
     * @param eigPointOnRay 3 DOF point on the ray
     * @param fEpsilon Metric describing threshold for distinguishing from perfectly parallel planes to practically parallel
     * @return 3 DOF point, if no intersection returns nanf("1") vector
     */

    float fNDotU = eigPlaneNormal.dot(eigRayDirection);
    if (fNDotU < fEpsilon and fNDotU > -fEpsilon)
    {
        return {0,0,0};
    }

    Eigen::Vector3f eigW = eigPointOnRay - eigPointOnPlane;
    float fSi = (eigPlaneNormal * -1).dot(eigW) / fNDotU;
    Eigen::Vector3f eigN = eigRayDirection * fSi;
    Eigen::Vector3f eigO = eigW + eigN;

    Eigen::Vector3f eigPsi = eigO + eigPointOnPlane;
    return eigPsi;
}

vector<vector<Eigen::Vector3f>> LoadStlMesh(const string& strStlPath)
{
    /**
     * @brief Extract the facet vertices and normals from a .stl file
     * @param strStlPath path where the stl is located
     * @return a list of lists with the following data { {normal}, {v1}, {v2}, {v3}}
     */

    vector<vector<Eigen::Vector3f>> lstlstfFacets {};
    try
    {
        stl_reader::StlMesh <float, unsigned int> cStlMesh {strStlPath.c_str()};

        for (size_t nTriangles = 0; nTriangles < cStlMesh.num_tris(); ++nTriangles)
        {
            vector<Eigen::Vector3f> lstfFacet {};
            Eigen::Vector3f eigNormal {cStlMesh.tri_normal(nTriangles)};
            lstfFacet.push_back(eigNormal);
            for (size_t nVertices = 0; nVertices < 3; ++nVertices)
            {
                Eigen::Vector3f eigVertex {cStlMesh.tri_corner_coords(nTriangles, nVertices)};
                lstfFacet.push_back(eigVertex);
            }
            lstlstfFacets.push_back(lstfFacet);
        }
    }
    catch(const std::exception& e)
    {
        lstlstfFacets = {};
    }

    return lstlstfFacets;

}

vector<vector<float>> GetMeshData(const vector<vector<Eigen::Vector3f>>& lstlstfSTLFacets)
{
    /**
     * @brief Calculates data about the facets for a given mesh
     * @param lstlstfSTLFacets Mesh container with facets definitions
     * @return List of data { AREA , COLLISION DISTANCE }
     */

    vector<vector<float>> lstlstfData {};

    for (const vector<Eigen::Vector3f>& lstfFacet : lstlstfSTLFacets)
    {
        vector<float> lstfData {};
        // calc the area
        // eigAB = eigVertex2 - eigVertex1
        Eigen::Vector3f eigAB = lstfFacet[2] - lstfFacet[1];
        Eigen::Vector3f eigAC = lstfFacet[3] - lstfFacet[1];

        // area from the 1/2 cross method
        float fArea = eigAB.cross(eigAC).norm() / 2;
        lstfData.push_back(fArea);
        lstlstfData.push_back(lstfData);
    }
    Eigen::Vector3f eigNan {0,0,0};

    for (unsigned i = 0; i < lstlstfSTLFacets.size(); i++)
    {
        // float fDistance = 0.0;
        for (unsigned j = 0; j < lstlstfSTLFacets.size(); j++)
        {
            if (lstlstfSTLFacets[i][0] == lstlstfSTLFacets[j][0] and lstlstfSTLFacets[i][1] == lstlstfSTLFacets[j][1] and lstlstfSTLFacets[i][2] == lstlstfSTLFacets[j][2] and lstlstfSTLFacets[i][3] == lstlstfSTLFacets[j][3])
                continue;

            // if (lstlstfSTLFacets[i][1] == lstlstfSTLFacets[j][1] or lstlstfSTLFacets[i][1] == lstlstfSTLFacets[j][2] or lstlstfSTLFacets[i][1] == lstlstfSTLFacets[j][3])
            //     continue;
            
            // if (lstlstfSTLFacets[i][2] == lstlstfSTLFacets[j][1] or lstlstfSTLFacets[i][2] == lstlstfSTLFacets[j][2] or lstlstfSTLFacets[i][2] == lstlstfSTLFacets[j][3])
            //     continue;

            // if (lstlstfSTLFacets[i][3] == lstlstfSTLFacets[j][1] or lstlstfSTLFacets[i][3] == lstlstfSTLFacets[j][2] or lstlstfSTLFacets[i][3] == lstlstfSTLFacets[j][3])
            //     continue;

            // get the ray plane intersection using first vertex
            Eigen::Vector3f eigRayAB = lstlstfSTLFacets[i][2] - lstlstfSTLFacets[i][1];
            Eigen::Vector3f eigRayIntermediate = (eigRayAB * 0.5) + lstlstfSTLFacets[i][1];
            Eigen::Vector3f eigRayIC = lstlstfSTLFacets[i][3] - eigRayIntermediate;
            Eigen::Vector3f eigRayCenterIshPoint = (eigRayIC * 0.5) + lstlstfSTLFacets[i][3];

            Eigen::Vector3f eigPlaneAB = lstlstfSTLFacets[j][2] - lstlstfSTLFacets[j][1];
            Eigen::Vector3f eigPlaneIntermediate = (eigPlaneAB * 0.5) + lstlstfSTLFacets[j][1];
            Eigen::Vector3f eigPlaneIC = lstlstfSTLFacets[j][3] - eigPlaneIntermediate;
            Eigen::Vector3f eigPlaneCenterIshPoint = (eigPlaneIC * 0.5) + lstlstfSTLFacets[j][3];

            Eigen::Vector3f eigIntersection = RayPlaneIntersection(lstlstfSTLFacets[j][0], eigPlaneCenterIshPoint, lstlstfSTLFacets[i][0], eigRayCenterIshPoint);

            // Eigen::Vector3f eigPlaneMean = {(lstlstfSTLFacets[j][1][0] + lstlstfSTLFacets[j][2][0] +  lstlstfSTLFacets[j][3][0]) / 3, (lstlstfSTLFacets[j][1][1] + lstlstfSTLFacets[j][2][1] +  lstlstfSTLFacets[j][3][1]) / 3, (lstlstfSTLFacets[j][1][2] + lstlstfSTLFacets[j][2][2] +  lstlstfSTLFacets[j][3][2]) / 3};
            // Eigen::Vector3f eigRayMean = {(lstlstfSTLFacets[i][1][0] + lstlstfSTLFacets[i][2][0] +  lstlstfSTLFacets[i][3][0]) / 3, (lstlstfSTLFacets[i][1][1] + lstlstfSTLFacets[i][2][1] +  lstlstfSTLFacets[i][3][1]) / 3, (lstlstfSTLFacets[i][1][2] + lstlstfSTLFacets[i][2][2] +  lstlstfSTLFacets[i][3][2]) / 3};
            // Eigen::Vector3f eigIntersection = RayPlaneIntersection(lstlstfSTLFacets[j][0], lstlstfSTLFacets[j][1], lstlstfSTLFacets[i][0], eigRayMean);
            // cout << "intersection " << i << "," << j << " : (" << eigIntersection[0] << "," << eigIntersection[1] << "," << eigIntersection[2] << ")" << endl;
            // if (eigIntersection.hasNaN() == true)
            if (eigIntersection == eigNan)
            {
                continue;
            }
            if (CheckInsideFacet(eigIntersection, lstlstfSTLFacets[j][1], lstlstfSTLFacets[j][2], lstlstfSTLFacets[j][3], lstlstfData[j][0]) == false)
                continue;

            Eigen::Vector3f eigPN = eigIntersection - eigRayCenterIshPoint;
            // Eigen::Vector3f eigPN = eigIntersection - eigRayMean;
            float fDistance = eigPN.norm();
            lstlstfData[i].push_back(fDistance);
            break;
        }
        if (lstlstfData[i].size() < 2)
                lstlstfData[i].push_back(0.0);
    }

    return lstlstfData;
}

vector<vector<Eigen::Vector3f>> ReverseForwardKinematics(CAngleAxisPose clstfRobotPose_w, const CAngleAxisPose& clstfTcp_f, const vector<Eigen::Vector3f>& lsteigPointsOfInterest, const vector<float>& lstfJointAngles)
{
    /**
     * @brief Calculates the points of interests given for the given pose and joint angles
     * @param clstfRobotPose_w Robot tool pose in world coordinates
     * @param clstfTcp_f Tcp pose in flange coordinates
     * @param lsteigPointsOfInterest 3 DOF points of interest on the joints to find from the flange
     * @param lstfJointAngles Angles of each joint, length should match the points of interest
     * @return a list of lists containing {{3D point, Direction unit vector between joints}}
     */

    vector<vector<Eigen::Vector3f>> lstlsteigData {};
    vector<Eigen::Vector3f> lsteigPoints {};
    for (int i = 0; i < lsteigPointsOfInterest.size(); i++)
    {
        CAngleAxisPose clstfPoint_f {};
        // CAngleAxisPose clstfRotation {0,0,0,0,0,lstfJointAngles[i]};

        Eigen::Vector3f eigRotatedPoint_f = RodriguesRotation(lsteigPointsOfInterest[i], {0,0,-lstfJointAngles[i]});
        
        // CAngleAxisPose clstfRotatedPoint_f {};
        // clstfRotatedPoint_f = clstfRotatedPoint_f.FromFrame(clstfRotation.Frame().inverse() * clstfPoint_f.Frame());

        clstfPoint_f = clstfTcp_f;
        clstfPoint_f[0] -= eigRotatedPoint_f[0];
        clstfPoint_f[1] -= eigRotatedPoint_f[1];
        clstfPoint_f[2] -= eigRotatedPoint_f[2];
        CAngleAxisPose clstfPoint_w {};
        clstfPoint_w = clstfPoint_w.FromFrame(clstfRobotPose_w.Frame() * clstfPoint_f.Frame().inverse());

        lstlsteigData.push_back({{clstfPoint_w[0], clstfPoint_w[1], clstfPoint_w[2]}});
    }
    Eigen::Vector3f lstfFirstVec = lstlsteigData[1][0] - lstlsteigData[0][0];
    lstfFirstVec *= 1/lstfFirstVec.norm();
    lstlsteigData[0].push_back(lstfFirstVec);

    for (int i = 1; i < lstlsteigData.size(); i++)
    {
        Eigen::Vector3f lstfVec = lstlsteigData[i][0] - lstlsteigData[i-1][0];
        lstfVec *= 1/lstfVec.norm();
        lstlsteigData[i].push_back(lstfVec);
    }
    return lstlsteigData;
}

Eigen::Vector3f RodriguesRotation(const Eigen::Vector3f& eigObject, const Eigen::Vector3f& eigDirection)
{
    /**
     * @brief Performs rotation following Rodrigues equation
     * @param eigObject 3DOF point/vector to be rotated around its origin
     * @param eigRotation Rotation vector in degrees
     * @return 3DOF rotated point/vector
     */
    Eigen::Vector3f eigRotation_rad = eigDirection * PI_F/180.0;
    float fRotationAngle_rad = eigRotation_rad.norm();
    if (fRotationAngle_rad == 0)
        return eigObject;
    
    Eigen::Vector3f eigUnitVec = eigRotation_rad / eigRotation_rad.norm();

    Eigen::Vector3f eigRotatedVec = (eigObject * cosf(fRotationAngle_rad)) + (eigUnitVec.cross(eigObject) * sinf(fRotationAngle_rad)) + (eigUnitVec * ((1-cosf(fRotationAngle_rad)) * eigUnitVec.dot(eigObject)));

    return eigRotatedVec;
}

vector<Eigen::Vector3f> GeneratePointCloudCylinder(const Eigen::Vector3f& eigCylinderCenter, const Eigen::Vector3f& eigDirection, const float& fRadius, const float& fLength, const int& nSlices, const int& nPointsOnSlice)
{
    /**
     * @brief Generates a point cloud model around the given center point in the given direction with the given parameters
     * @param eigCylinderCenter 3DOF point center of the cylinder in world coordinates
     * @param eigDirection 3DOF unit vector
     * @param fRadius Radius of the cylinder in mm
     * @param fLength Length of the cylinder in mm
     * @param nSlices Number of slices to generate points at
     * @param nPointsOnSlice Number of points to generate on the slice
     * @return List of eigen vectors describing a point
     */
    float fLengthStep = fLength / nSlices;
    float fAngleStep = 2 * PI_F / nPointsOnSlice;
    float fHeight = -fLength/2;

    vector<Eigen::Vector3f> lsteigPoints {};
    lsteigPoints.push_back({0,0,fHeight});

    while (fHeight <= fLength/2)
    {
        float fAngle = 0;
        lsteigPoints.push_back({fRadius*cosf(fAngle), fRadius*sinf(fAngle), fHeight});
        while (fAngle < 0.99*2*PI_F)
        {
            fAngle += fAngleStep;
            lsteigPoints.push_back({fRadius*cosf(fAngle), fRadius*sinf(fAngle), fHeight});
        }
        fHeight += fLengthStep;
    }

    lsteigPoints.push_back({0,0,fLength/2});

    Eigen::Vector3f eigRotation = RotationBetweenVectors({0,0,1}, eigDirection);

    for (Eigen::Vector3f& eigPoint : lsteigPoints)
    {
        eigPoint = RodriguesRotation(eigPoint, eigRotation);
        eigPoint = eigPoint + eigCylinderCenter;
    }

    return lsteigPoints;
}

Eigen::Vector3f RotationBetweenVectors(const Eigen::Vector3f& eigVec1, const Eigen::Vector3f& eigVec2)
{
    /**
     * @brief Calculates the rotation vector to rotate vector 1 to vector 2
     * @param eigVec1 3D vector 1
     * @param eigVec2 3D vector 2
     * @return Rotation vector
     */
    float fCos = eigVec1.dot(eigVec2);
    Eigen::Vector3f eigCross = eigVec1.cross(eigVec2);
    float fSin = eigCross.norm();

    if (fSin == 0)
        return {0,0,0};
    
    Eigen::Vector3f eigUnit = eigCross / fSin;
    float fU = eigUnit[0];
    float fV = eigUnit[1];
    float fW = eigUnit[2];

    Eigen::Matrix3f eigRotation;
    eigRotation << 
        (fU*fU*(1-fCos) + fCos), (fU*fV*(1-fCos) - fW*fSin), (fU*fW*(1-fCos) + fV*fSin),
        (fV*fU*(1-fCos) + fW*fSin), (fV*fV*(1-fCos) + fCos), (fV*fW*(1-fCos) - fU*fSin),
        (fW*fU*(1-fCos) - fV*fSin), (fW*fV*(1-fCos) + fU*fSin), (fW*fW*(1-fCos) + fCos);

    return RotationMatrixToVector(eigRotation);
}

Eigen::Vector3f RotationMatrixToVector(const Eigen::Matrix3f& eigRotation)
{
    /**
     * @brief Converts a rotation matrix to angle axis rotation vector representation
     * @param eigRotation 3x3 rotation matrix
     * @return 3x1 rotation vector
     */
    float fTrace = eigRotation.trace();
    float fTheta = acosf((fTrace-1)/2);

    float fDenom = 2*sinf(fTheta);

    if (fDenom == 0 or fDenom == PI_F)
        fDenom = 1;

    float fRx = (180.0/PI_F) * fTheta * (1/fDenom) * (eigRotation(2,1) - eigRotation(1,2));
    float fRy = (180.0/PI_F) * fTheta * (1/fDenom) * (eigRotation(0,2) - eigRotation(2,0));
    float fRz = (180.0/PI_F) * fTheta * (1/fDenom) * (eigRotation(1,0) - eigRotation(0,1));

    return {fRx, fRy, fRz};
}

vector<vector<float>> LoadPath(const string& strPath, const int& nCoordinateLength)
{
    /**
     * @brief Load a 2D path from a file
     * @param strPath Location relative to location of calling program
     * @param nCoordinateLength Number of indices in a coordinate, default is 3
     * @note Assumes a third coordinate indicating tangent angle
     * @note path should be defined in origin direction of operation [0,1,0]
     * @return List of lists of path in units mm and rad
     */

    ifstream strmPathFile {strPath, ios::in};
    stringstream strmPath {};
    strmPath << strmPathFile.rdbuf();
    vector<vector<float>> lstlstfCoordinates {};

    try
    {
        while (strmPath.good())
        {
            string strCoordinates;
            getline(strmPath, strCoordinates);
            if (strmPath.eof())
                break;

            stringstream strmCoordinates {strCoordinates.c_str()};
            vector<float> lstfCoordinates;
            for (int i = 0; i < nCoordinateLength; i++)
            {
                string strValue;
                getline(strmCoordinates, strValue, ',');
                lstfCoordinates.push_back(stof(strValue));
            }
            lstlstfCoordinates.push_back(lstfCoordinates);
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

    return lstlstfCoordinates;
}

vector<CAngleAxisPose> Generate6DofPath(const vector<vector<float>>& lstlstf2DPath, const CAngleAxisPose& lstfPathOffset, const CAngleAxisPose& lstfTilt)
{
    /**
     * @brief Generates 6Dof angle axis points for a given path
     * @param lstlstf2D coordinates of path
     * @param lstfPathOffset Optional arg defining how much to offset the original path in mm and deg
     * @param lstfTilt Optional arg defining how to tilt the path in deg
     * @note assumes 3 vec indices for a tangent angle in rad, current generation method is using tangent angle as rz
     */

    vector<CAngleAxisPose> lstclstfPath {};

    for (vector<float> lstfVec : lstlstf2DPath)
    {
        vector<float> lstfFullVec {lstfVec[0], lstfVec[1], 0, 0, 0, RadiansToDegrees(lstfVec[2])};
        CAngleAxisPose clstfPoint {};
        clstfPoint = lstfFullVec;
        clstfPoint = clstfPoint.PreMultiply(lstfPathOffset);
        clstfPoint = clstfPoint.PostMultiply(lstfTilt);

        cout << "Offset: " << "(" << lstfPathOffset[0] << "," << lstfPathOffset[1] << "," << lstfPathOffset[2] << "," << lstfPathOffset[3] << ","<< lstfPathOffset[4] << ","<< lstfPathOffset[5] << ")" << endl;
        cout << "Point: " << "(" << clstfPoint[0] << "," << clstfPoint[1] << "," << clstfPoint[2] << "," << clstfPoint[3] << ","<< clstfPoint[4] << ","<< clstfPoint[5] << ")" << endl;

        lstclstfPath.push_back(clstfPoint);
    }

    return lstclstfPath;
}

vector<CAngleAxisPose> Generate6DofTab(const vector<vector<float>>& lstlstf2DPath, const CAngleAxisPose& lstfPathOffset, const CAngleAxisPose& lstfTilt)
{
    /**
     * @brief Generates 6Dof angle axis points for a given path
     * @param lstlstf2D coordinates of path
     * @param lstfPathOffset Optional arg defining how much to offset the original path in mm and deg
     * @param lstfTilt Optional arg defining how to tilt for tabs in deg
     * @note assumes 3 vec indices for a tangent angle in rad, current generation method is using tangent angle as rz
     * @note this is specific to making movements for tabs
     */

    vector<CAngleAxisPose> lstclstfPath {};
    // CAngleAxisPose clstfRotation = {0, 0, 0, 11.775261, -28.427995, 43.945872};

    for (vector<float> lstfVec : lstlstf2DPath)
    {
        vector<float> lstfFullVec {lstfVec[0], lstfVec[1], 0, 0, 0, RadiansToDegrees(lstfVec[2])};
        CAngleAxisPose clstfPoint {};
        clstfPoint = lstfFullVec;
        clstfPoint = clstfPoint.PreMultiply(lstfPathOffset);
        clstfPoint = clstfPoint.PostMultiply(lstfTilt);

        lstclstfPath.push_back(clstfPoint);
    }

    return lstclstfPath;
}