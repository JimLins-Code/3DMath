#pragma once
//////////////////////////////////////////////////////////////////////////
//
// 3Dͼ����ѧ����
// 
// Matrix4x3.h - ��Matrix4x3������
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

		// ��Ϊ��λ����
		void identity();

		// ����ƽ�Ʋ��ֵĽӿ�
		void zeroTranslation();
		void setTranslation(const Vector3& d);
		void setupTranslation(const Vector3& d);
		Vector3 getTranslation()const ;

		// ����3x3���ֵ�����ʽֵ
		float determinant()const;

		// ��Ϊ�����
		void inverse();

		// ���ռ�->�ֲ��ռ�ı任���󣬼ٶ��ֲ��ռ���ָ����λ�úͷ�λ���ķ�λ������ŷ���ǻ�����ת����
		void setupLocalToParent(const Vector3& pos,const EulerAngle& orient);
		void setupLocalToParent(const Vector3& pos, const RotationMatrix& orient);
		void setupParentToLocal(const Vector3& pos, const EulerAngle& orient);
		void setupParentToLocal(const Vector3& pos, const RotationMatrix& orient);

		Vector3 getPositionFromParentToLocalMatrix()const;
		Vector3 getPositionFromLocalToParentMatrix()const;

		// ����������ת
		void setupRotate(const Vector3& axis, float theta);

		// ������ת����
		void fromQuaternion(const Quaternion& q);

		// ������������ ��Vector3(1.0,1.0,0.5)��ʾ��Z������0.5��
		void setupScale(const Vector3& s);
		// ������������k��
		void setupScalAlongAxis(const Vector3& axis, float k);

		// �б����
		void setupShear(int axis, float s, float t);

		// ͶӰ����ͶӰƽ���ԭ��
		void setupProjectionMatrix(const Vector3& n);

		// �������
		void setupReflectionMatrix(int axis, float k = 0.0f);
		// ����������ƽ��ķ������
		void setupReflectionMatrix(const Vector3& n);

};