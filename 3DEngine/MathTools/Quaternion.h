#pragma once
#include "Vector3.h"
#include "EulerAngles.h"

class Quaternion
{
	public:
		float w, x, y, z;

		Quaternion() :w(1), x(0.0f), y(0.0f), z(0.0f) {};

		// ��Ϊ��λ��Ԫ��
		void identity()
		{
			w = 1.0f;
			x = y = z = 0.0f;
		}

		/*
			������ת��x��y��z�������ᣩ
		*/
		void setToRotateAboutX(float theta);
		void setToRotateAboutY(float theta);
		void setToRotateAboutZ(float theta);
		void setToRotateAboutAxis(const Vector3& axis, float theta);

		// ����ִ�С�����->����"��ת����Ԫ������λ��ŷ���ǵ���ʽ����
		void setToRotateObjectToInertial(const EulerAngle&& orientation);
		void setToRotateInertialToObject(const EulerAngle&& orientation);

		// ���

		Quaternion operator*(const Quaternion& a)const;

		Quaternion& operator*=(const Quaternion& a);

		// ����
		/*
			ͨ������¶������򻯵ģ���Ҫ��Ϊ�˷�ֹ��ֹ�����������������������Ư��
		*/
		void normalize();

		float getRotationAngle()const;
		Vector3 getRotationAxis()const;
};
// ȫ�ֵ�λ��Ԫ��
extern const Quaternion kQuaternionIndentity;

// ��Ԫ�����
extern float dotProduct(const Quaternion& a,const Quaternion& b);

// �������Բ�ֵ
extern Quaternion slerp(const Quaternion& q0, const Quaternion& q1, float t);

// ��⹲����Ԫ��
extern Quaternion conjugate(const Quaternion& q);

// ��Ԫ������
extern Quaternion pow(const Quaternion& q, float exponent);













