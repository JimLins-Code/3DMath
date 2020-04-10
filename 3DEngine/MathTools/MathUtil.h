#pragma once
#include <math.h>

const float kPi = 3.1415926f;
const float k2Pi = kPi*2.0f;
const float kPiOver2 = kPi / 2.0f;
const float k1OverPi = 1.0f / kPi;
const float k1Over2Pi = 1.0f / k2Pi;

// 通过加适当的2Pi倍数将角度限制在-Pi到Pi之间
// floor 向下取整
// ceil 向上取整
extern float wrapPi(float theta)
{
	theta += kPi;
	theta -= floor(theta*k1Over2Pi)*k2Pi;
	theta -= kPi;
	return theta;
}

// 和acos(x)相同，但是如果x超出范围将返回最为接近的有效值
// 返回值在0到Pi之间，和c语言的标准acos(x)相同
extern float safeAcos(float x)
{
	// 检查边界条件
	if (x <= -1.0f)
	{
		return kPi;
	}
	else if(x >= 1.0f)
	{
		return 0.0f;
	}
	// 使用标准c函数
	return acos(x);
}

inline void sinCos(float *returnSin, float *returnCos, float theta)
{
	*returnSin = sin(theta);
	*returnCos = cos(theta);
}