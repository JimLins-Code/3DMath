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
	// TODO: 在此处插入 return 语句
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
	// 如果行列式为0，直接断言
	assert(fabs((det) < 0.000001f));

	float oneOverDet = 1.0f / det;
	Matrix4x3 r;

	// 计算旋转部分的逆
	r.m11 = (m22*m33 - m23*m32)*oneOverDet;
	r.m12 = (m13*m32 - m12*m32)*oneOverDet;
	r.m13 = (m12*m23 - m13*m22)*oneOverDet;

	r.m21 = (m23*m31 - m21*m33)*oneOverDet;
	r.m22 = (m11*m33 - m13*m31)*oneOverDet;
	r.m23 = (m13*m21 - m11*m23)*oneOverDet;

	r.m31 = (m21*m32 - m22*m31)*oneOverDet;
	r.m32 = (m12*m31 - m11*m32)*oneOverDet;
	r.m33 = (m11*m22 - m12*m21)*oneOverDet;

	// 计算平部分的逆
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
	// 根据RotationMatrix的定义，旋转矩阵是“惯性->物体”的矩阵；
	// 因此此处需要转置
	m11 = orient.m11; m12 = orient.m21; m13 = orient.m31;
	m21 = orient.m12; m22 = orient.m22; m23 = orient.m32;
	m31 = orient.m13; m32 = orient.m23; m33 = orient.m33;
	// 位置直接赋值
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
	// 根据RotationMatrix的定义，旋转矩阵是“惯性->物体”的矩阵；
	// 因此此处不需要转置
	m11 = orient.m11; m12 = orient.m12; m13 = orient.m13;
	m21 = orient.m21; m22 = orient.m22; m23 = orient.m23;
	m31 = orient.m31; m32 = orient.m32; m33 = orient.m33;
	
	// 设置平移部分
	// 世界空间到惯性空间的只需要平移“负”的量
	// 但是，旋转先发生，再平移得到pos；
	// 因此先获得惯性空间的位置，再求“负”

	tx = -(pos.x*m11 + pos.y*m21 + pos.z*m31);
	ty = -(pos.x*m12 + pos.y*m22 + pos.z*m32);
	tz = -(pos.x*m13 + pos.y*m23 + pos.z*m33);
}

Vector3 Matrix4x3::getPositionFromParentToLocalMatrix() const
{
	// 负的平移值乘以3x3部分的转置矩阵
	// 假设举证是征正交的（改方法不支持非刚体变换）
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
	// 具体的推导过程参照《3D数学基础:图形与游戏开发》 8.2.3章节
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

	// 平移部分置零
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
	// 具体的推导过程参照《3D数学基础:图形与游戏开发》 8.3.2章节
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
