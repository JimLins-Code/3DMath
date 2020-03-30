#pragma once
#include <cmath>

class Vector3
{
	public:
		// 成员变量
		float x, y, z;

		Vector3():x(0.0f),y(0.0f),z(0.0f) {};

		Vector3(float nx, float ny, float nz):x(nx),y(ny),z(nz) {};
		// 复制构造函数
		Vector3(const Vector3 &vec) :x(vec.x), y(vec.y), z(vec.z) {};
		// 重载赋值运算符
		Vector3& operator= (const Vector3& vec)
		{
			x = vec.x;
			y = vec.y;
			z = vec.z;
			return *this;
		}

		// 重载+=，-=，*=，/=
		Vector3& operator+=(const Vector3& vec)
		{
			x += vec.x;
			y += vec.y;
			z += vec.z;
			return *this;
		}

		Vector3& operator-=(const Vector3& vec)
		{
			x -= vec.x;
			y -= vec.y;
			z -= vec.z;
			return *this;
		}
		Vector3& operator*=(float k)
		{
			x *= k;
			y *= k;
			z *= k;
			return *this;
		}
		Vector3& operator/=(float k)
		{
			float oneOverK = 1.0f / k;
			x *= oneOverK;
			y *= oneOverK;
			z *= oneOverK;
			return *this;
		}

		// 重载比较运算符
		bool operator== (const Vector3& vec) const
		{
			return (x == vec.x) && (y == vec.y) && (z == vec.z);
		}
		// 重载取反符号
		Vector3 operator- ()const
		{
			return Vector3(-x, -y, -z);
		}
		// 重载相加符号
		Vector3 operator+ (const Vector3& vec)
		{
			return Vector3(x + vec.x, y + vec.y, z + vec.z);
		}
		// 乘常数
		Vector3 operator* (float k)const
		{
			return Vector3(x*k, y*k, z*k);
		}
		// 除常数
		Vector3 operator/(float k)
		{
			float oneOverk = 1.0f / k;
			//return *this*oneOverk;
			return Vector3(x*oneOverk, y*oneOverk, z*oneOverk);
		}
		// 向量点乘
		float operator*(const Vector3& vec)
		{
			return x*vec.x + y*vec.y + z*vec.z;
		}
		// 向量叉乘

		Vector3 CrossProduct(const Vector3& v)const
		{
			return Vector3(
				y*v.z - z*v.y
				, z*v.x - x*v.z
				, x*v.y - y*v.x);
		}

		/*向量运算*/
		void ToZero()
		{
			x = y = z = 0.0f;
		}
		// 标准化
		bool  normalize()
		{
			float magSq = x*x + y*y + z*z;
			if (magSq > 0.0f)
			{
				float OneOverMag = 1.0f / sqrt(magSq);
				x *= OneOverMag;
				y *= OneOverMag;
				z *= OneOverMag;
				return true;
			}
			return false;
		}

};

//////////////////////////////////////////////////////////////////////////
//
//全局变量
//
//////////////////////////////////////////////////////////////////////////

extern const Vector3 kZeroVector;

inline float vector3Mag(const Vector3& vec)
{
	return sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
}

inline Vector3 Vector3CrossProduct(const Vector3& vecA, const Vector3& vecB)
{
	return Vector3(
		vecA.y*vecB.z - vecA.z*vecB.y
		, vecA.z*vecB.x - vecA.x*vecB.z
		, vecA.x*vecB.y - vecA.y*vecB.x);
}

inline float Vector3Distance(const Vector3& v1, const Vector3& v2)
{
	float dx = v1.x - v2.x;
	float dy = v1.y - v2.y;
	float dz = v1.z - v2.z;
	return sqrt(dx*dx + dy*dy + dz*dz);
}