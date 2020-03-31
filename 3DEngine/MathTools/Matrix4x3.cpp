#include "pch.h"
#include "Matrix4x3.h"

#include <assert.h>
#include "Vector3.h"
#include "MathUtil.h"
#include "Quaternion.h"
#include "RotationMatrix.h"

Matrix4x3::Matrix4x3()
{
	identity();
}

Matrix4x3::~Matrix4x3()
{
}

Vector3 Matrix4x3::operator*(const Vector3 & p)
{
	return Vector3();
}

Matrix4x3 Matrix4x3::operator*(const Matrix4x3 & m)
{
	return Matrix4x3();
}

Matrix4x3 & Matrix4x3::operator*=(const Matrix4x3 & m)
{
	// TODO: �ڴ˴����� return ���
	*this *= m;
	return *this;
}

void Matrix4x3::identity()
{
	m11 = 1.0f; m12 = 0.0f; m13 = 0.0f;
	m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
	m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;
	tx = 0.0f; ty = 0.0f; tz = 0.0f;
}

void Matrix4x3::zeroTranslation()
{
	tx = ty = tz = 0.0f;
}

void Matrix4x3::setTranslation(const Vector3 & d)
{
	tx = d.x;
	ty = d.y;
	tz = d.z;
}

void Matrix4x3::setupTranslation(const Vector3 & d)
{
	identity();
	tx = d.x;
	ty = d.y;
	tz = d.z;
}

Vector3 Matrix4x3::getTranslation() const
{
	return Vector3(tx,ty,tz);
}

float Matrix4x3::determinant() const
{
	return m11*(m22*m33 - m23*m32)
		+ m12*(m23*m31 - m21*m33)
		+ m13*(m21*m32 - m22*m32);
}


void Matrix4x3::inverse()
{
	float det = determinant();
	// �������ʽΪ0��ֱ�Ӷ���
	assert(fabs((det) < 0.000001f));

	float oneOverDet = 1.0f / det;
	Matrix4x3 r;

	// ������ת���ֵ���
	r.m11 = (m22*m33 - m23*m32)*oneOverDet;
	r.m12 = (m13*m32 - m12*m32)*oneOverDet;
	r.m13 = (m12*m23 - m13*m22)*oneOverDet;

	r.m21 = (m23*m31 - m21*m33)*oneOverDet;
	r.m22 = (m11*m33 - m13*m31)*oneOverDet;
	r.m23 = (m13*m21 - m11*m23)*oneOverDet;

	r.m31 = (m21*m32 - m22*m31)*oneOverDet;
	r.m32 = (m12*m31 - m11*m32)*oneOverDet;
	r.m33 = (m11*m22 - m12*m21)*oneOverDet;

	// ����ƽ���ֵ���
	r.tx = -(tx*r.m11 + ty*r.m21 + tz*r.m31);
	r.ty = -(tx*r.m12 + ty*r.m22 + tz*r.m32);
	r.tz = -(tx*r.m13 + ty*r.m23 + tz*r.m33);
}

void Matrix4x3::setupLocalToParent(const Vector3 & pos, const EulerAngle & orient)
{
	RotationMatrix rotate;
	rotate.setup(orient);
	setupLocalToParent(pos, rotate);
}

void Matrix4x3::setupLocalToParent(const Vector3 & pos, const RotationMatrix & orient)
{
	// ����RotationMatrix�Ķ��壬��ת�����ǡ�����->���塱�ľ���
	// ��˴˴���Ҫת��
	m11 = orient.m11; m12 = orient.m21; m13 = orient.m31;
	m21 = orient.m12; m22 = orient.m22; m23 = orient.m32;
	m31 = orient.m13; m32 = orient.m23; m33 = orient.m33;
	// λ��ֱ�Ӹ�ֵ
	tx = pos.x; ty = pos.y; tz = pos.z;
}

void Matrix4x3::setupParentToLocal(const Vector3 & pos, const EulerAngle & orient)
{
	RotationMatrix rotate;
	rotate.setup(orient);
	setupParentToLocal(pos, rotate);
}

void Matrix4x3::setupParentToLocal(const Vector3 & pos, const RotationMatrix & orient)
{
	// ����RotationMatrix�Ķ��壬��ת�����ǡ�����->���塱�ľ���
	// ��˴˴�����Ҫת��
	m11 = orient.m11; m12 = orient.m12; m13 = orient.m13;
	m21 = orient.m21; m22 = orient.m22; m23 = orient.m23;
	m31 = orient.m31; m32 = orient.m32; m33 = orient.m33;
	
	// ����ƽ�Ʋ���
	// ����ռ䵽���Կռ��ֻ��Ҫƽ�ơ���������
	// ���ǣ���ת�ȷ�������ƽ�Ƶõ�pos��
	// ����Ȼ�ù��Կռ��λ�ã����󡰸���

	tx = -(pos.x*m11 + pos.y*m21 + pos.z*m31);
	ty = -(pos.x*m12 + pos.y*m22 + pos.z*m32);
	tz = -(pos.x*m13 + pos.y*m23 + pos.z*m33);
}

Vector3 Matrix4x3::getPositionFromParentToLocalMatrix() const
{
	// ����ƽ��ֵ����3x3���ֵ�ת�þ���
	// �����֤���������ģ��ķ�����֧�ַǸ���任��
	return Vector3(
		-(tx*m11+ty*m12+tz*m13),
		-(tx*m21+ty*m22+tz*m23),
		-(tx*m31+ty*m32+tz*m33)
	);
}

Vector3 Matrix4x3::getPositionFromLocalToParentMatrix() const
{
	return Vector3(tx,ty,tz);
}

void Matrix4x3::setupRotate(const Vector3 & axis, float theta)
{
	//assert(fabs(axis*axis - 1.0f) < 0.01f);
	// ������Ƶ����̲��ա�3D��ѧ����:ͼ������Ϸ������ 8.2.3�½�
	float sin, cos;
	sinCos(&sin, &cos, theta);

	float a = 1.0f - cos;
	float ax = a*axis.x;
	float ay = a*axis.y;
	float az = a*axis.z;

	m11 = ax*axis.x + cos;
	m12 = ax*axis.y + axis.z*sin;
	m13 = ax*axis.z - axis.y*sin;

	m21 = ay*axis.x - axis.z*sin;
	m22 = ay*axis.y + cos;
	m23 = ay*axis.z + axis.x*sin;

	m31 = az*axis.x + axis.y*sin;
	m32 = az*axis.y - axis.x*sin;
	m33 = az*axis.z + cos;

	// ƽ�Ʋ�������
	tx = ty = tz = 0.0f;
}

void Matrix4x3::fromQuaternion(const Quaternion & q)
{

}

void Matrix4x3::setupScale(const Vector3 & s)
{
	identity();
	m11 = s.x; m22 = s.y; m33 = s.z;
}

void Matrix4x3::setupScalAlongAxis(const Vector3 & axis, float k)
{
	// ������Ƶ����̲��ա�3D��ѧ����:ͼ������Ϸ������ 8.3.2�½�
	//assert(fabs(axis*axis - 1.0f) < 0.001f);
	float a = k - 1.0f;
	float ax = a*axis.x;
	float ay = a*axis.y;
	float az = a*axis.z;

	m11 = ax*axis.x + 1.0f;
	m22 = ay*axis.y + 1.0f;
	m32 = az*axis.z + 1.0f;

	m12 = m21 = ax*axis.y;
	m13 = m31 = ax*axis.z;
	m23 = m32 = ay*axis.z;
}

void Matrix4x3::setupShear(int axis, float s, float t)
{
}

void Matrix4x3::setupProjectionMatrix(const Vector3 & n)
{
}

void Matrix4x3::setupReflectionMatrix(int axis, float k)
{
}

void Matrix4x3::setupReflectionMatrix(const Vector3 & n)
{
}
