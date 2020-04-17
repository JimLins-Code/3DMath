#pragma once
#include "Vector3.h"

class Vertex
{
	public:
		Vertex() {
			setDefaults();
		};
		~Vertex() {};

		Vector3 m_vec;
		Vector3 m_normal;

		float u, v;

		int mark;

		void setDefaults()
		{
			m_vec = kZeroVector;
			m_normal = kZeroVector;
			u = v = .0f;
			mark = 0;
		}
};

class Triangle
{
	public:
		Triangle(){
			setDefaults();
		}
		~Triangle() {};
		void setDefaults()
		{

		}

		// �涥��
		struct Vert {

			int index;
			float u, v;
		};

		Vert m_vert[3];

		//���淨����
		Vector3 m_normal;

		// �����������һ����
		int part;

		// �����б������
		int material;

		// ���߱���
		int mark;

		// �Ƿ����˻�������-----ͬһ����ʹ�ó���һ��
		bool isDegenerate() const
		{
		
		}

};


class EditTriMesh
{
	public:
};