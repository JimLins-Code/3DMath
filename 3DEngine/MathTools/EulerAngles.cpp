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
		//  ����������£��������ƴ�ֱ�����תֵ����heading;
		heading += bank;
		bank = 0.0f;
	}
	else
	{
		// ������������½�bankת�������Ƽ���
		bank = wrapPi(bank);
	}
	// ��headingת�������Ƽ���
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


