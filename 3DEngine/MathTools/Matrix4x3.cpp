#include "pch.h"
#include "Matrix4x3.h"

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
	return 0.0f;
}


void Matrix4x3::inverse()
{
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
	return Vector3();
}

Vector3 Matrix4x3::getPositionFromLocalToParentMatrix() const
{
	return Vector3();
}

void Matrix4x3::setupRotate(const Vector3 & axis, float theta)
{
	//assert(fabs(axis*axis - 1.0f) < 0.01f);
	float sin, cos;
	sinCos(&sin, &cos, theta);

}

void Matrix4x3::fromQuaternion(const Quaternion & q)
{

}

void Matrix4x3::setupScale(const Vector3 & s)
{
}

void Matrix4x3::setupScalAlongAxis(const Vector3 & axis, float k)
{
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
