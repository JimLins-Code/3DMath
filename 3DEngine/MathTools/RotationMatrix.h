#pragma once
//////////////////////////////////////////////////////////////////////////
//
// 3D图形数学基础
// 
// RotationMatrix.h - 类RotationMatrix的声明
// 
// 该类为指标是旋转的矩阵，用3x3矩阵形式表示。使用方式：
//	1，初始化矩阵，使用表示方位的EulerAngle/quaternion初始化。注意区分“物体->惯性空间"或相反
//	2，用矩阵执行向量的旋转
// 
//////////////////////////////////////////////////////////////////////////

#include "EulerAngles.h"

class Vector3;
class EurlerAngle;
class Quaternion;


class RotationMatrix
{
	public:
		float m11, m12, m13;
		float m21, m22, m23;
		float m31, m32, m33;

		RotationMatrix();
		// 置为单位矩阵
		void identity();

		// 根据指定的方位构造矩阵
		void setup(const EulerAngle& orientation);

		// 根据四元数构造矩阵，假设改四元数参数代表指定方向的变换
		void fromInertialToObjectQuaternion(const Quaternion& q);
		void fromObjectToInertialQuaternion(const Quaternion& q);

		// 执行旋转
		Vector3 inertialToObject(const Vector3& v)const;
		Vector3 objectToInertial(const Vector3& v)const;

};