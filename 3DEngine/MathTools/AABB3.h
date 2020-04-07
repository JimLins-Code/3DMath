#pragma once
#include "Vector3.h"
#include "Matrix4x3.h"

class AABB3
{
	public:
		AABB3() : min(kZeroVector), max(kZeroVector){};
		~AABB3() {};

		Vector3 min;
		Vector3 max;

		Vector3 size()const;
		Vector3 center()const;

		// 重置包围盒
		void empty();

		// 添加矩形边界框的点
		void add(const Vector3 &p);
		
		// 添加另一个边界
		void add(const AABB3& box);

		// 变换矩形边界框，重新计算包围盒
		void setToTransFormedBox(const AABB3& box, const Matrix4x3& m);

		bool isEmpty()const;
};