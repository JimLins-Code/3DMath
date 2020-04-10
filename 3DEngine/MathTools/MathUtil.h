#pragma once
#include <math.h>

const float kPi = 3.1415926f;
const float k2Pi = kPi*2.0f;
const float kPiOver2 = kPi / 2.0f;
const float k1OverPi = 1.0f / kPi;
const float k1Over2Pi = 1.0f / k2Pi;

// ͨ�����ʵ���2Pi�������Ƕ�������-Pi��Pi֮��
// floor ����ȡ��
// ceil ����ȡ��
extern float wrapPi(float theta)
{
	theta += kPi;
	theta -= floor(theta*k1Over2Pi)*k2Pi;
	theta -= kPi;
	return theta;
}

// ��acos(x)��ͬ���������x������Χ��������Ϊ�ӽ�����Чֵ
// ����ֵ��0��Pi֮�䣬��c���Եı�׼acos(x)��ͬ
extern float safeAcos(float x)
{
	// ���߽�����
	if (x <= -1.0f)
	{
		return kPi;
	}
	else if(x >= 1.0f)
	{
		return 0.0f;
	}
	// ʹ�ñ�׼c����
	return acos(x);
}

inline void sinCos(float *returnSin, float *returnCos, float theta)
{
	*returnSin = sin(theta);
	*returnCos = cos(theta);
}