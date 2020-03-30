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
		// ��Ϊ��λ����
		void identity();

		// ����ָ���ķ�λ�������
		void setup(const EulerAngle& orientation);

		// ������Ԫ��������󣬼������Ԫ����������ָ������ı任
		void fromInertialToObjectQuaternion(const Quaternion& q);
		void fromObjectToInertialQuaternion(const Quaternion& q);

		Vector3 inertialToObject(const Vector3& v)const;
		Vector3 objectToInertial(const Vector3& v)const;

};