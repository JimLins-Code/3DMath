#include "pch.h"
#include "AABB3.h"

#include <assert.h> 
#include<algorithm>

Vector3 AABB3::size() const
{
	return (maxP - minP);
}

Vector3 AABB3::center() const
{
	return (maxP + minP)*0.5;
}

void AABB3::empty()
{
	const float kBigNumber = 1e37;
	minP.x = minP.y = minP.z = kBigNumber;
	maxP.x = maxP.y = maxP.z = -kBigNumber;
}

bool AABB3::isEmpty() const
{
	return minP.x > maxP.x || minP.z > maxP.z || minP.z > maxP.z;
}

void AABB3::add(const Vector3 & p)
{
	if (p.x < minP.x) minP.x = p.x;
	if (p.x > maxP.x) maxP.x = p.x;
	if (p.y < minP.y) minP.y = p.y;
	if (p.y > maxP.y) maxP.y = p.y;
	if (p.z < minP.z) minP.z = p.z;
	if (p.z > maxP.z) maxP.z = p.z;
}

void AABB3::add(const AABB3 & box)
{
	if (box.minP.x < minP.x) minP.x = box.minP.x;
	if (box.maxP.x > maxP.x) maxP.x = box.maxP.x;
	if (box.minP.y < minP.y) minP.y = box.minP.y;
	if (box.maxP.y > maxP.y) maxP.y = box.maxP.y;
	if (box.minP.z < minP.z) minP.z = box.minP.z;
	if (box.maxP.z > maxP.z) maxP.z = box.maxP.z;
}

void AABB3::setTransFormedBox( const Matrix4x3 & m)
{
	if (isEmpty())
	{
		empty();
		return;
	}

	minP = maxP = m.getTranslation();
	/*
		整体的思路就是：a+=分支最小数,a是最小数
						b+=分支最大值，b是最大值
	*/

	if (m.m11 > 0.0f)
	{
		minP.x += m.m11*minP.x;
		maxP.x += m.m11*maxP.x;
	}
	else
	{
		minP.x += m.m11*maxP.x;
		maxP.x += m.m11*minP.x;
	}

	if (m.m12 > 0.0f)
	{
		minP.y += m.m12*minP.x;
		maxP.y += m.m12*maxP.x;
	}
	else
	{
		minP.y += m.m12*maxP.x;
		maxP.y += m.m12*minP.x;
	}

	if (m.m13 > 0.0f)
	{
		minP.z += m.m13*minP.x;
		maxP.z += m.m13*maxP.x;
	}
	else
	{
		minP.z += m.m13*maxP.x;
		maxP.z += m.m13*minP.x;
	}

	if (m.m21 > 0.0f)
	{
		minP.x += m.m21*minP.y;
		maxP.x += m.m21*maxP.y;
	}
	else
	{
		minP.x += m.m21*maxP.y;
		maxP.x += m.m21*minP.y;
	}

	if (m.m22 > 0.0f)
	{
		minP.y += m.m22*minP.y;
		maxP.y += m.m22*maxP.y;
	}
	else
	{
		minP.y += m.m22*maxP.y;
		maxP.y += m.m22*minP.y;
	}

	if (m.m23 > 0.0f)
	{
		minP.z += m.m23*minP.y;
		maxP.z += m.m23*maxP.y;
	}
	else
	{
		minP.z += m.m23*maxP.y;
		maxP.z += m.m23*minP.y;
	}

	if (m.m31 > 0.0f)
	{
		minP.x += m.m31*minP.z;
		maxP.x += m.m31*maxP.z;
	}
	else
	{
		minP.x += m.m31*maxP.z;
		maxP.x += m.m31*minP.z;
	}

	if (m.m32 > 0.0f)
	{
		minP.y += m.m32*minP.z;
		maxP.y += m.m32*maxP.z;
	}
	else
	{
		minP.y += m.m32*maxP.z;
		maxP.y += m.m32*minP.z;
	}

	if (m.m33 > 0.0f)
	{
		minP.z += m.m33*minP.z;
		maxP.z += m.m33*maxP.z;
	}
	else
	{
		minP.z += m.m33*maxP.z;
		maxP.z += m.m33*minP.z;
	}
}

bool AABB3::isContains(const Vector3 & p) const
{
	return (p.x >= minP.x)&&(p.y >= minP.y)&&(p.z >= minP.z)
		&&(p.x <= maxP.x) && (p.y <= maxP.y) && (p.z <= maxP.z);
}

Vector3 AABB3::closestPointTo(const Vector3 & p) const
{
	Vector3 ret;
	// x
	if (p.x < minP.x)
	{
		ret.x = minP.x;
	}
	else if (p.x > maxP.x)
	{
		ret.x = maxP.x;
	}
	else
	{
		ret.x = p.x;
	}
	// y
	if (p.y < minP.y)
	{
		ret.y = minP.y;
	}
	else if (p.y > maxP.y)
	{
		ret.y = maxP.y;
	}
	else
	{
		ret.y = p.y;
	}
	// z
	if (p.z < minP.z)
	{
		ret.z = minP.z;
	}
	else if (p.z > maxP.z)
	{
		ret.z = maxP.z;
	}
	else
	{
		ret.z = p.z;
	}

	return ret;
}


bool AABB3::intersectionSphere(const Vector3 & center, float radius)
{
	Vector3 closestPoint = closestPointTo(center);
	return Vector3Distance(center,closestPoint) <= radius;
}

float AABB3::rayIntersect(const Vector3 & rayOrg, const Vector3 & rayDelta, Vector3 * returnNormal) const
{
	const float kNoIntersection = 1e30f;
	bool inside = true;
	float xt, xn;
	if (rayOrg.x < minP.x)
	{
		xt = minP.x - rayOrg.x;
		if (xt > rayDelta.x) return kNoIntersection;
		xt /= rayDelta.x;
		inside = false;
		xn = -1.0f;
	}
	else if (rayOrg.x > maxP.x)
	{
		xt = maxP.x - rayOrg.x;
		if (xt < rayDelta.x) return kNoIntersection;
		xt /= rayDelta.x;
		inside = false;
		xn = 1.0f;
	}
	else
	{
		xt = -1.0f;
	}

	float yt, yn;
	if (rayOrg.y < minP.y)
	{
		yt = minP.y - rayOrg.y;
		if (yt > rayDelta.y) return kNoIntersection;
		yt /= rayDelta.y;
		inside = false;
		yn = -1.0f;
	}
	else if (rayOrg.y > maxP.y)
	{
		yt = maxP.y - rayOrg.y;
		if (yt < rayDelta.y) return kNoIntersection;
		yt /= rayDelta.y;
		inside = false;
		yn = 1.0f;
	}
	else
	{
		yt = -1.0f;
	}

	float zt, zn;
	if (rayOrg.z < minP.z)
	{
		zt = minP.z - rayOrg.z;
		if (zt > rayDelta.z) return kNoIntersection;
		zt /= rayDelta.z;
		inside = false;
		zn = -1.0f;
	}
	else if (rayOrg.z > maxP.z)
	{
		zt = maxP.z - rayOrg.z;
		if (zt < rayDelta.z) return kNoIntersection;
		zt /= rayDelta.z;
		inside = false;
		zn = 1.0f;
	}
	else
	{
		zt = -1.0f;
	}

	if (inside)
	{
		if (returnNormal != NULL)
		{
			*returnNormal = -rayDelta;
			returnNormal->normalize();
		}
		return 0.0f;
	}
	// 选择最远端的平面，也就是发生相交的地方
	int which = 0;
	float t = xt;
	if (yt > t)
	{
		which = 1;
		t = yt;
	}
	if (zt > t)
	{
		which = 2;
		t = zt;
	}
	switch (which)
	{
		case 0:// 和yz平面相交
		{
			float y = rayOrg.y + rayDelta.y*t;
			if (y<minP.y || y>maxP.y) return kNoIntersection;
			float z = rayOrg.z + rayDelta.z*t;
			if (z<minP.z || z>maxP.z) return kNoIntersection;
			if (returnNormal != NULL)
			{
				returnNormal->x = xn;
				returnNormal->y = 0.0f;
				returnNormal->z = 0.0f;
			}
			break;
		}
		case 1:// 和xz平面相交
		{
			float x = rayOrg.x + rayDelta.x*t;
			if (x<minP.x || x>maxP.x) return kNoIntersection;
			float z = rayOrg.z + rayDelta.z*t;
			if (z<minP.z || z>maxP.z) return kNoIntersection;
			if (returnNormal != NULL)
			{
				returnNormal->x = 0.0f;
				returnNormal->y = yn;
				returnNormal->z = 0.0f;
			}
			break;
		}
		case 2:// 和xy平面相交
		{
			float x = rayOrg.x + rayDelta.x*t;
			if (x<minP.x || x>maxP.x) return kNoIntersection;
			float y = rayOrg.y + rayDelta.y*t;
			if (y<minP.y || y>maxP.y) return kNoIntersection;
			if (returnNormal != NULL)
			{
				returnNormal->x = 0.0f;
				returnNormal->y = 0.0f;
				returnNormal->z = zn;
			}
			break;
		}
		default:
			return kNoIntersection;
			break;
	}
	return t;
}

int AABB3::classifyPlane(const Vector3 & n, float d) const
{
	// 检查法向量，计算最小和最大的D值。
	float minD, maxD;
	if (n.x > 0.0)
	{
		minD = n.x*minP.x; maxD = n.x*maxP.x;
	}
	else
	{
		minD = n.x*maxP.x; maxD = n.x*minP.x;
	}

	if (n.y>0.0)
	{
		minD += n.y*minP.y; maxD += n.y*maxP.y;
	}
	else
	{
		minD += n.y*maxP.y; maxD += n.y*minP.y;
	}
	if (n.z > 0.0)
	{
		minD += n.z*minP.z; maxD += n.z*maxP.z;
	}
	else
	{
		minD += n.z*maxP.z; maxD += n.z*minP.z;
	}

	if (minD >= d)
	{
		// 完全在正面
		return 1.0;
	}
	if (maxD <= d)
	{
		// 完全在背面
		return -1.0;
	}
	return 0;
}

float AABB3::intersectPlane(const Vector3 & n, float planeD, const Vector3 & dir) const
{
	const float kNoIntersection = 1e30;

	assert(fabs(n*n - 1.0) < 0.01f);
	assert(fabs(dir*dir - 1.0) < 0.01f);
	float dot = n*dir;
	if (dot >= 0)
	{
		return kNoIntersection;
	}
	float minD, maxD;
	if (n.x > 0.0)
	{
		minD = n.x*minP.x; maxD = n.x*maxP.x;
	}
	else
	{
		minD = n.x*maxP.x; maxD = n.x*minP.x;
	}

	if (n.y > 0.0)
	{
		minD += n.y*minP.y; maxD += n.y*maxP.y;
	}
	else
	{
		minD += n.y*maxP.y; maxD += n.y*minP.y;
	}

	if (n.z > 0.0)
	{
		minD += n.z*minP.z; maxD += n.z*maxP.z;
	}
	else
	{
		minD += n.z*maxP.z; maxD += n.z*minP.z;
	}

	// 检测是否包围盒是否已经在平面的另一面。
	if (maxD <= planeD)
	{
		return kNoIntersection;
	}

	float t = (planeD - minD) / dot;
	if (t < 0.0f) // 已经穿过了？
	{
		return 0.0f;
	}
	// 如果 t > 1，则表示包围盒未能到达平面。需要调用者自行判断 
	return t;
}

bool intersectAABBs(const AABB3 & box1, const AABB3 & box2, AABB3 * boxIntersect)
{
	if (box1.minP.x > box2.maxP.x) return false;
	if (box1.maxP.x < box2.minP.x) return false;
	if (box1.minP.y > box2.maxP.y) return false;
	if (box1.maxP.y < box2.minP.y) return false;
	if (box1.minP.z > box2.maxP.z) return false;
	if (box1.maxP.y < box2.minP.y) return false;

	if (boxIntersect != NULL)
	{
		boxIntersect->minP.x = max(box1.minP.x, box2.minP.x);
		boxIntersect->maxP.x = min(box1.maxP.x, box2.maxP.x);
		boxIntersect->minP.y = max(box1.minP.y, box2.minP.y);
		boxIntersect->maxP.y = min(box1.maxP.y, box2.maxP.y);
		boxIntersect->minP.z = max(box1.minP.z, box2.minP.z);
		boxIntersect->maxP.z = min(box1.maxP.z, box2.maxP.z);
	}
	return true;
}

float intersectMovingAABB(const AABB3 & stationaryBox, const AABB3 & movingBox, const Vector3 & d)
{

	const float kNoIntersection = 1e30;
	float tEnter = 0.0f;
	float tLeave = 1.0f;
	if (d.x == 0.0f)
	{
		if ((stationaryBox.minP.x >= movingBox.maxP.x)||(stationaryBox.maxP.x <= movingBox.minP.x))
		{
			return kNoIntersection;
		}
	}
	else
	{
		float oneOverD = 1.0f / d.x;
		float xEnter = (stationaryBox.minP.x - movingBox.maxP.x)*oneOverD;
		float xLeave = (stationaryBox.maxP.x - movingBox.minP.x)*oneOverD;
		if (xEnter > xLeave)
		{
			std::swap(xEnter, xLeave);
		}
		if (xEnter > tEnter) tEnter = xEnter;
		if (xLeave < tLeave) tLeave = xLeave;
		if (tEnter > tLeave)
		{
			return kNoIntersection;
		}
	}

	if (d.y == 0.0f)
	{
		if ((stationaryBox.minP.y >= movingBox.maxP.y) || (stationaryBox.maxP.y <= movingBox.minP.y))
		{
			return kNoIntersection;
		}
	}
	else
	{
		float oneOverD = 1.0f / d.y;
		float yEnter = (stationaryBox.minP.y - movingBox.maxP.y)*oneOverD;
		float yLeave = (stationaryBox.maxP.y - movingBox.minP.y)*oneOverD;
		if (yEnter > yLeave)
		{
			std::swap(yEnter, yLeave);
		}
		if (yEnter > tEnter) tEnter = yEnter;
		if (yLeave < tLeave) tLeave = yLeave;
		if (tEnter > tLeave)
		{
			return kNoIntersection;
		}
	}

	if (d.z == 0.0f)
	{
		if ((stationaryBox.minP.z >= movingBox.maxP.z) || (stationaryBox.maxP.z <= movingBox.minP.z))
		{
			return kNoIntersection;
		}
	}
	else
	{
		float oneOverD = 1.0f / d.z;
		float zEnter = (stationaryBox.minP.z - movingBox.maxP.z)*oneOverD;
		float zLeave = (stationaryBox.maxP.z - movingBox.minP.z)*oneOverD;
		if (zEnter > zLeave)
		{
			std::swap(zEnter, zLeave);
		}
		if (zEnter > tEnter) tEnter = zEnter;
		if (zLeave < tLeave) tLeave = zLeave;
		if (tEnter > tLeave)
		{
			return kNoIntersection;
		}
	}

	// 有相交，返回相交点的参数值
	return tEnter;
}
