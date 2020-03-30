#pragma once
#include "Vector3.h"
#include "EulerAngles.h"

class Quaternion
{
	public:
		float w, x, y, z;

		Quaternion() :w(1), x(0.0f), y(0.0f), z(0.0f) {};

		// 置为单位四元数
		void identity()
		{
			w = 1.0f;
			x = y = z = 0.0f;
		}

		/*
			绕轴旋转（x，y，z，任意轴）
		*/
		void setToRotateAboutX(float theta);
		void setToRotateAboutY(float theta);
		void setToRotateAboutZ(float theta);
		void setToRotateAboutAxis(const Vector3& axis, float theta);

		// 构造执行“物体->惯性"旋转的四元数，方位以欧拉角的形式给出
		void setToRotateObjectToInertial(const EulerAngle&& orientation);
		void setToRotateInertialToObject(const EulerAngle&& orientation);

		// 叉乘

		Quaternion operator*(const Quaternion& a)const;

		Quaternion& operator*=(const Quaternion& a);

		// 正则化
		/*
			通常情况下都是正则化的，主要是为了防止防止误差扩大，连续操作导致数据漂移
		*/
		void normalize();

		float getRotationAngle()const;
		Vector3 getRotationAxis()const;
};
// 全局单位四元数
extern const Quaternion kQuaternionIndentity;

// 四元数点成
extern float dotProduct(const Quaternion& a,const Quaternion& b);

// 球面线性插值
extern Quaternion slerp(const Quaternion& q0, const Quaternion& q1, float t);

// 求解共轭四元数
extern Quaternion conjugate(const Quaternion& q);

// 四元数求幂
extern Quaternion pow(const Quaternion& q, float exponent);













