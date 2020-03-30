#include "pch.h"
#include "RotationMatrix.h"

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

}

void RotationMatrix::fromInertialToObjectQuaternion(const Quaternion & q)
{

}

void RotationMatrix::fromObjectToInertialQuaternion(const Quaternion & q)
{

}

Vector3 RotationMatrix::inertialToObject(const Vector3 & v) const
{
	return Vector3();
}

Vector3 RotationMatrix::objectToInertial(const Vector3 & v) const
{
	return Vector3();
}
