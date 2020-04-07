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

		// ���ð�Χ��
		void empty();

		// ��Ӿ��α߽��ĵ�
		void add(const Vector3 &p);
		
		// �����һ���߽�
		void add(const AABB3& box);

		// �任���α߽�����¼����Χ��
		void setToTransFormedBox(const AABB3& box, const Matrix4x3& m);

		bool isEmpty()const;
};