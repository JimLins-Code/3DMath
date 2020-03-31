#pragma once
//////////////////////////////////////////////////////////////////////////
//
// 3D图形数学基础
// 
// Matrix4x3.h - 类Matrix4x3的声明
//  
//////////////////////////////////////////////////////////////////////////

class Vector3;
class EulerAngle;
class Quaternion;
class RotationMatrix;

class Matrix4x3
{
	public:
		Matrix4x3();
		~Matrix4x3();

		float m11, m12, m13;
		float m21, m22, m23;
		float m31, m32, m33;
		float tx, ty, tz;


		Vector3 operator*(const Vector3& p);
		Matrix4x3 operator*(const Matrix4x3& m);
		Matrix4x3& operator*=(const Matrix4x3& m);

		// 置为单位矩阵
		void identity();

		// 访问平移部分的接口
		void zeroTranslation();
		void setTranslation(const Vector3& d);
		void setupTranslation(const Vector3& d);
		Vector3 getTranslation()const ;

		// 计算3x3部分的行列式值
		float determinant()const;

		// 置为逆矩阵
		void inverse();

		// 父空间->局部空间的变换矩阵，假定局部空间在指定的位置和方位，改方位可以是欧拉角或者旋转矩阵
		void setupLocalToParent(const Vector3& pos,const EulerAngle& orient);
		void setupLocalToParent(const Vector3& pos, const RotationMatrix& orient);
		void setupParentToLocal(const Vector3& pos, const EulerAngle& orient);
		void setupParentToLocal(const Vector3& pos, const RotationMatrix& orient);

		Vector3 getPositionFromParentToLocalMatrix()const;
		Vector3 getPositionFromLocalToParentMatrix()const;

		// 绕任意轴旋转
		void setupRotate(const Vector3& axis, float theta);

		// 构造旋转矩阵
		void fromQuaternion(const Quaternion& q);

		// 沿坐标轴缩放 如Vector3(1.0,1.0,0.5)表示沿Z轴缩放0.5倍
		void setupScale(const Vector3& s);
		// 沿任意轴缩放k倍
		void setupScalAlongAxis(const Vector3& axis, float k);

		// 切变矩阵
		void setupShear(int axis, float s, float t);

		// 投影矩阵，投影平面过原点
		void setupProjectionMatrix(const Vector3& n);

		// 反射矩阵
		void setupReflectionMatrix(int axis, float k = 0.0f);
		// 构造沿任意平面的反射矩阵
		void setupReflectionMatrix(const Vector3& n);

};