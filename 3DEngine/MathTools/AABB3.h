#pragma once
#include "Vector3.h"
#include "Matrix4x3.h"


class AABB3
{
	public:
		AABB3() : minP(kZeroVector), maxP(kZeroVector){};
		~AABB3() {};

		Vector3 minP;
		Vector3 maxP;

		Vector3 size()const;
		Vector3 center()const;

		// 重置包围盒
		void empty();

		// 是否是单位包围盒
		bool isEmpty()const;

		// 添加矩形边界框的点
		void add(const Vector3 &p);
		
		// 添加另一个边界
		void add(const AABB3& box);

		// 变换矩形边界框，重新计算包围盒
		void setTransFormedBox( const Matrix4x3& m);

		// 是否包含某个点
		bool isContains(const Vector3& p)const;

		// 返回距离AABB上的最近点
		Vector3 closestPointTo(const Vector3& p)const;

		// 与球面相交检测
		bool intersectionSphere(const Vector3& center, float radius);

		// 射线相交检测,如果相交的话返回0到1之间的参数值。否则返回值大于1的任意参数。
		// 快速射线与矩形边界框的相交性检测方法
		float rayIntersect(const Vector3& rayOrg, const Vector3& rayDelta, Vector3* returnNormal = 0)const;

		// 判断包围盒在平面的哪一边
		// <0 包围盒在平面的背面
		// >0 包围盒在平面的正面
		// 0 包围盒和平面相交
		int classifyPlane(const Vector3& n, float d)const;

		// 和平面的动态相交检测
		float intersectPlane(const Vector3& n, float planeD, const Vector3& dir)const;
}; // AABB3

/*
	全局函数
*/
// 两个AABB3相交性测试，如果相交返回true,还可以返回相交的AABB3
bool intersectAABBs(const AABB3& box1, const AABB3& box2, AABB3* boxIntersect = 0);

// 返回运动AABB和静止的AABB相交的时候的参数点，如果不相交则返回的数值 >1
float intersectMovingAABB(const AABB3& stationaryBox, const AABB3& movingBox, const Vector3& d);