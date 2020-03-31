#pragma once

#include <string>

class Quaternion;
class Matrix4x3;
class RotationMatrix;

class EulerAngle
{
	public:
	// ������Ա����
		float heading;
		float pitch;
		float bank;

		EulerAngle() {};
		EulerAngle(float h, float p, float b) :heading(h), pitch(p), bank(b) {	
		};

		// ��Ա��������
		void identity()
		{
			heading = pitch = bank = 0.0f;
		}

		// �任Ϊ���Ƽ�ŷ����
		void canonize();

		// ����Ԫ��ת����ŷ����
		// �������Ԫ��Ϊ����->���Ի��߹���->�������Ԫ��
		// ��������ϵ�����᷽������������ϵ��ͬ��λ������������λ���غϡ�����⣺ĳ����Ĺ�������ϵ�����������ϵֻ��Ҫ��ת��������

		void formObjectToInertualQuaternion(const Quaternion& q);
		void fromInertialToObjectQuaternion(const Quaternion& q);

		// �Ӿ���ŷ����
		// ����ľ���Ϊ����->�����������->�����ת������
		// ƽ�Ʋ��ֱ�ʡ��
		// �ú���ʵ�ְѡ����嵽��������ϵ�ı任���󡱵���ת����ת��Ϊŷ����
		void fromObjectToWorldMatrix(const Matrix4x3& m);
		void fromWorldToObjectMatrix(const Matrix4x3& m);

		 // ����ת����ת����ŷ����
		void fromRotationMatrix(const RotationMatrix& m);
};

// ȫ�ֵĵ�λŷ����
extern const EulerAngle kEulerAnglesIdentity;