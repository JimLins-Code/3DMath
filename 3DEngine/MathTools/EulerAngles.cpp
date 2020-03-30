#include "pch.h"
#include "EulerAngles.h"

#include "MathUtil.h"

#include <math.h>




void EulerAngle::canonize()
{
	//
	pitch = wrapPi(pitch);

	if (pitch < -kPiOver2)
	{
		pitch = -kPi - pitch;
		heading += kPi;
		bank += kPi;
	}
	else if (pitch>kPiOver2)
	{
		pitch = kPi - pitch;
		heading += kPi;
		bank += kPi;
	}
	 
	if (fabs(pitch) > kPiOver2 - 1e-4)
	{
		//  万向锁情况下，将所有绕垂直轴的旋转值赋予heading;
		heading += bank;
		bank = 0.0f;
	}
	else
	{
		// 非万向锁情况下将bank转换到限制集中
		bank = wrapPi(bank);
	}
	// 将heading转换到限制集中
	heading = wrapPi(heading);
}

void EulerAngle::formObjectToInertualQuaternion(const Quaternion & q)
{
}

void EulerAngle::fromInertialToObjectQuaternion(const Quaternion & q)
{
}

void EulerAngle::fromObjectToWorldMatrix(const Matrix4x3 & m)
{
}

void EulerAngle::fromWorldToObjectMatrix(const Matrix4x3 & m)
{
}

void EulerAngle::fromRotationMatrix(const RotationMatrix & m)
{
}


