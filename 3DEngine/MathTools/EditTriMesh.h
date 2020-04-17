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

		// 面顶点
		struct Vert {

			int index;
			float u, v;
		};

		Vert m_vert[3];

		//表面法向量
		Vector3 m_normal;

		// 属于网格的那一部分
		int part;

		// 材质列表的索引
		int material;

		// 工具变量
		int mark;

		// 是否是退化三角形-----同一顶点使用超过一次
		bool isDegenerate() const
		{
		
		}

};


class EditTriMesh
{
	public:
};