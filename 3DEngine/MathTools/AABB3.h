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

		// ���ð�Χ��
		void empty();

		// �Ƿ��ǵ�λ��Χ��
		bool isEmpty()const;

		// ��Ӿ��α߽��ĵ�
		void add(const Vector3 &p);
		
		// �����һ���߽�
		void add(const AABB3& box);

		// �任���α߽�����¼����Χ��
		void setTransFormedBox( const Matrix4x3& m);

		// �Ƿ����ĳ����
		bool isContains(const Vector3& p)const;

		// ���ؾ���AABB�ϵ������
		Vector3 closestPointTo(const Vector3& p)const;

		// �������ཻ���
		bool intersectionSphere(const Vector3& center, float radius);

		// �����ཻ���,����ཻ�Ļ�����0��1֮��Ĳ���ֵ�����򷵻�ֵ����1�����������
		// ������������α߽����ཻ�Լ�ⷽ��
		float rayIntersect(const Vector3& rayOrg, const Vector3& rayDelta, Vector3* returnNormal = 0)const;

		// �жϰ�Χ����ƽ�����һ��
		// <0 ��Χ����ƽ��ı���
		// >0 ��Χ����ƽ�������
		// 0 ��Χ�к�ƽ���ཻ
		int classifyPlane(const Vector3& n, float d)const;

		// ��ƽ��Ķ�̬�ཻ���
		float intersectPlane(const Vector3& n, float planeD, const Vector3& dir)const;
}; // AABB3

/*
	ȫ�ֺ���
*/
// ����AABB3�ཻ�Բ��ԣ�����ཻ����true,�����Է����ཻ��AABB3
bool intersectAABBs(const AABB3& box1, const AABB3& box2, AABB3* boxIntersect = 0);

// �����˶�AABB�;�ֹ��AABB�ཻ��ʱ��Ĳ����㣬������ཻ�򷵻ص���ֵ >1
float intersectMovingAABB(const AABB3& stationaryBox, const AABB3& movingBox, const Vector3& d);