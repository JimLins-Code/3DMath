#pragma once

#include <string>

class Quaternion;
class Matrix4x3;
class RotationMatrix;

class EulerAngle
{
	public:
	// 三个成员对象
		float heading;
		float pitch;
		float bank;

		EulerAngle() {};
		EulerAngle(float h, float p, float b) :heading(h), pitch(p), bank(b) {	
		};

		// 成员操作函数
		void identity()
		{
			heading = pitch = bank = 0.0f;
		}

		// 变换为限制集欧拉角
		void canonize();

		// 从四元数转换到欧拉角
		// 输入的四元数为物体->惯性或者惯性->物体的四元数
		// 惯性坐标系方向轴方向与世界坐标系相同，位置与物体所在位置重合。可理解：某物体的惯性坐标系与物体的坐标系只需要旋转就能做到

		void formObjectToInertualQuaternion(const Quaternion& q);
		void fromInertialToObjectQuaternion(const Quaternion& q);

		// 从矩阵到欧拉角
		// 输入的矩阵为物体->世界或者世界->物体的转换矩阵
		// 平移部分被省略
		// 该函数实现把“物体到世界坐标系的变换矩阵”的旋转部分转换为欧拉角
		void fromObjectToWorldMatrix(const Matrix4x3& m);
		void fromWorldToObjectMatrix(const Matrix4x3& m);

		 // 从旋转矩阵转换到欧拉角
		void fromRotationMatrix(const RotationMatrix& m);
};

// 全局的单位欧拉量
extern const EulerAngle kEulerAnglesIdentity;