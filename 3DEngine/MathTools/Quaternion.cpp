#include "pch.h"
#include "Quaternion.h"

#include "MathUtil.h"

/*
	������ת��x��y��z�������ᣩ
*/

void Quaternion::setToRotateAboutX(float theta)
{
	// ������
	float thetaOver2 = theta * 0.5f;
	w = cos(thetaOver2);
	x = sin(thetaOver2);
	y = 0.;
	z = 0.;
}

void Quaternion::setToRotateAboutY(float theta)
{
	// ������
	float thetaOver2 = theta * 0.5f;
	w = cos(thetaOver2);
	x = 0.;
	y = sin(thetaOver2);
	z = 0.;
}

void Quaternion::setToRotateAboutZ(float theta)
{
	// ������
	float thetaOver2 = theta * 0.5f;
	w = cos(thetaOver2);
	x = 0.;
	y = 0.;
	z = sin(thetaOver2);
}


void Quaternion::setToRotateAboutAxis(const Vector3 & axis, float theta)
{
	// ���ǵ�λ����
	//assert(fabs(vector3Mag(axis) - 1.0f) < 0.001f);

	// ������
	float thetaOver2 = theta * 0.5f;
	// ������sinֵ
	float sinThetaOver2 = sin(thetaOver2);

	w = cos(thetaOver2);
	x = axis.x * sinThetaOver2;
	y = axis.y * sinThetaOver2;
	z = axis.z * sinThetaOver2;
}



void Quaternion::setToRotateObjectToInertial(const EulerAngle && orientation)
{
	float sp, sb, sh;
	float cp, cb, ch;
	sinCos(&sp, &cp, orientation.pitch*0.5f);
	sinCos(&sb, &cb, orientation.bank*0.5f);
	sinCos(&sh, &ch, orientation.heading*0.5f);

	w = ch*cp*cb + sh*sp*sb;
	x = ch*sp*cb + sh*cp*sb;
	y = -ch*sp*sb + sh*cp*cb;
	z = -sh*sp*cb + ch*cp*sb;

}

void Quaternion::setToRotateInertialToObject(const EulerAngle && orientation)
{
	float sp, sb, sh;
	float cp, cb, ch;
	sinCos(&sp, &cp, orientation.pitch*0.5f);
	sinCos(&sb, &cb, orientation.bank*0.5f);
	sinCos(&sh, &ch, orientation.heading*0.5f);

	w = ch*cp*cb + sh*sp*sb;
	x = -ch*sp*cb - sh*cp*sb;
	y = ch*sp*sb - sh*cp*cb;
	z = sh*sp*cb - ch*cp*sb;
}

/*
	�����ĳ˷�
*/
Quaternion Quaternion::operator*(const Quaternion & a) const
{
	Quaternion result;
	result.w = w*a.w - x*a.x - y*a.y - z*a.z;
	result.x = w*a.x + x*a.w + y*a.z - z*a.y;
	result.y = w*a.y + y*a.w + z*a.x - x*a.z;
	result.z = w*a.z + z*a.w + x*a.y - y*a.x ;
	return result;
}

Quaternion & Quaternion::operator*=(const Quaternion & a)
{
	// TODO: �ڴ˴����� return ���
	*this = *this*a;
	return *this;
}

void Quaternion::normalize()
{
	float mag = sqrt(w*w + x*x + y*y + z*z);
	//��ֹ����
	if (mag > 0.0f)
	{
		float onOverMag = 1.0f / mag;
		w *= onOverMag;
		x *= onOverMag;
		y *= onOverMag;
		z *= onOverMag;
	}
	else
	{
	//	assert(false);
		identity();
	}
}

float Quaternion::getRotationAngle() const
{
	float thetaOver2 = safeAcos(w);

	return thetaOver2 * 2.0f;
}

Vector3 Quaternion::getRotationAxis() const
{
	float sinThetaOver2Sq = 1.0f - w*w;
	if (sinThetaOver2Sq <= 0.0f)
	{
		return Vector3(1.f,0.,0.);
	}
	float oneOverSinThetaOver2 = 1.0f / sqrt(sinThetaOver2Sq);
	return Vector3(x*oneOverSinThetaOver2, y*oneOverSinThetaOver2, z*oneOverSinThetaOver2);
}

float dotProduct(const Quaternion & a, const Quaternion & b)
{
	return a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
}

Quaternion slerp(const Quaternion & q0, const Quaternion & q1, float t)
{
	if (t <= 0.0f) return q0;
	if (t >= 1.0f) return q1;
	//�õ�˼�����Ԫ���нǵ�cosֵ
	float cosOmega = dotProduct(q0, q1);

	float q1w = q1.w;
	float q1x = q1.x;
	float q1y = q1.y;
	float q1z = q1.z;
	// ������Ϊ����ʹ��-q1;
	// ��Ԫ��q��-q������ͬ����ת�����ǿ��ܲ�����ͬ��slerp���㣬�˴��Ĳ�����ʾѡ����ǽ�����ת
	if (cosOmega < 0.0f)
	{
		q1w = -q1w;
		q1x = -q1x;
		q1y = -q1y;
		q1z = -q1z;
		cosOmega = -cosOmega;
	}
	// ������λ��Ԫ���ĵ�˽��Ӧ��<=1.0f
	//assert(cosOmega > 1.0f);
	float k0, k1;
	if (cosOmega>0.9999f)
	{
		// ������λ�ǳ��ӽ�����ʹ�����Բ�ֵ����ֹ����
		k0 = 1.0f - t;
		k1 = t;
	}
	else
	{
		// �������ǹ�ʽsin^2(omega)+cos^2(omega) = 1����sinֵ
		float sinOmega = sqrt(1.0f - cosOmega*cosOmega);
		// ����sin��cos����Ƕ�
		float omega = atan2(sinOmega, cosOmega);

		float oneOverSinOmega = 1.0 / sinOmega;
		// �����ֵ����
		k0 = sin((1.0 - t)*omega)*oneOverSinOmega;
		k1 = sin(t*omega)*oneOverSinOmega;
	}

	Quaternion result;
	result.x = k0*q0.x + k1*q1.x;
	result.y = k0*q0.y + k1*q1.y;
	result.z = k0*q0.z + k1*q1.z;
	result.w = k0*q0.w + k1*q1.w;
	return result;
}

Quaternion conjugate(const Quaternion & q)
{
	Quaternion result;
	// ��ת�Ƕ���ͬ
	result.w = q.w;
	// ��ת���෴
	result.x = -q.x;
	result.y = -q.y;
	result.z = -q.z;

	return result;
}

Quaternion pow(const Quaternion & q, float exponent)
{
	// ��鵥λ��Ԫ������ֹ����
	if (fabs(q.w)>0.999f)
	{
		return q;
	}
	// ��ȡ���alpha(alpha = theta/2)
	float alpha = acos(q.w);
	// �����µİ��ֵ
	float newAlpha = alpha*exponent;
	
	Quaternion result;
	// �����µ�wֵ
	result.w = cos(newAlpha);
	// �����µ�x,y,zֵ
	float mult = sin(newAlpha) / sin(alpha);

	result.x = q.x*mult;
	result.y = q.y*mult;
	result.z = q.z*mult;

	return result;
}
