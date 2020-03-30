#pragma once

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

		Vector3 inertialToObject(const Vector3& v)const;
		Vector3 objectToInertial(const Vector3& v)const;

};