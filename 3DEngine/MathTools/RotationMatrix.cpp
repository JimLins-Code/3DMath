#include "pch.h"
#include "RotationMatrix.h"

#include "MathUtil.h"
#include "EulerAngles.h"
#include "Vector3.h"
#include "Quaternion.h"

RotationMatrix::RotationMatrix()
:m11(1.),m12(0.),m13(0.)
,m21(0.),m22(1.),m23(0.)
,m31(0.),m32(0.),m33(1.)
{
}

void RotationMatrix::identity()
{
	m11 = 1.; m12 = 0.; m13 = 0.;
	m21 = 0.; m22 = 1.; m23 = 0.;
	m31 = 0.; m32 = 0.; m33 = 1.;
}

void RotationMatrix::setup(const EulerAngle & orientation)
{
	// 先计算欧拉角表示的sin和cos值
	float sh, ch;
	float sp, cp;
	float sb, cb;
	sinCos(&sh, &ch, orientation.heading);
	sinCos(&sp, &cp, orientation.pitch);
	sinCos(&sb, &cb, orientation.bank);

	// 填充矩阵
	m11 = ch*cb + sh*sp*sb;
	m12 = -ch*sb + sh*sp*cb;
	m13 = sh*cp;
	m21 = sb*cp;
	m22 = cb*cp;
	m23 - sp;
	m31 = -sh*cb + ch*sp*sb;
	m32 = sb*sh + ch*sp*cb;
	m33 = ch*cp;
}

void RotationMatrix::fromInertialToObjectQuaternion(const Quaternion & q)
{
	m11 = 1.0f - 2.0f*(q.y*q.y + q.z*q.z);
	m12 = 2.0f*(q.x*q.y + q.w*q.z);
	m13 = 2.0f*(q.x*q.z - q.w*q.y);

	m21 = 2.0f*(q.x*q.y - q.w*q.z);
	m22 = 1.0f - 2.0f*(q.x*q.x + q.z*q.z);
	m23 = 2.0f*(q.y*q.z + q.w*q.x);

	m31 = 2.0f*(q.x*q.z + q.w*q.y);
	m32 = 2.0f*(q.y*q.z - q.w*q.x);
	m33 = 1.0f - 2.0f*(q.x*q.x + q.y*q.y);
}

void RotationMatrix::fromObjectToInertialQuaternion(const Quaternion & q)
{
	m11 = 1.0f - 2.0f*(q.y*q.y + q.z*q.z);
	m12 = 2.0f*(q.x*q.y - q.w*q.z);
	m13 = 2.0f*(q.x*q.z + q.w*q.y);

	m21 = 2.0f*(q.x*q.y + q.w*q.z);
	m22 = 1.0f - 2.0f*(q.x*q.x + q.z*q.z);
	m23 = 2.0f*(q.y*q.z - q.w*q.x);

	m31 = 2.0f*(q.x*q.z - q.w*q.y);
	m32 = 2.0f*(q.y*q.z + q.w*q.x);
	m33 = 1.0f - 2.0f*(q.x*q.x + q.y*q.y);
}

Vector3 RotationMatrix::inertialToObject(const Vector3 & v) const
{
	// 这里设定向量为“行向量”

	return Vector3(
		m11*v.x+m21*v.y+m31*v.z,
		m12*v.x+m22*v.y+m32*v.z,
		m13*v.x+m23*v.y+m33*v.z
	);
}

Vector3 RotationMatrix::objectToInertial(const Vector3 & v) const
{
	// 乘以转置
	return Vector3(
		m11*v.x + m12*v.y + m13*v.z,
		m21*v.x + m22*v.y + m23*v.z,
		m31*v.x + m32*v.y + m33*v.z
	);
}
