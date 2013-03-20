/**
Barycentric coordinate
- �d�S���W�����߂�
- �A�����������������߂ɃN�����[���̌������g��
- �s�񎮂�0�ɂȂ����ꍇ�̑Ώ����K�v
*/

#include <cstdio>

// �񎟌��x�N�g��
class Vec2
{
public:
	Vec2()
	{}
	Vec2( float ax, float ay)
		:x(ax),y(ay)
	{}
	Vec2 operator - (const Vec2& other) const
	{
		return Vec2(x-other.x,y-other.y);
	}
	// �������
	float lengthSq() const
	{
		return x*x+y*y;
	}
	// ��_�ԋ���
	static float distance( const Vec2& lhs, const Vec2& rhs )
	{
		const float dx = lhs.x - rhs.x;
		const float dy = lhs.y - rhs.y;
		return dx*dx+dy*dy;
	}
	// ����
	static float dot( const Vec2& lhs, const Vec2& rhs )
	{
		return lhs.x * rhs.x + lhs.y * rhs.y;
	}

public:
	float x;
	float y;
};

// �O�����x�N�g��
class Vec3
{
public:
	Vec3()
	{}
	Vec3( float ax, float ay, float az)
		:x(ax),y(ay),z(az)
	{}
	Vec3 operator - (const Vec3& other) const
	{
		return Vec3(x-other.x,y-other.y,z-other.z);
	}
	// �������
	float lengthSq() const
	{
		return x*x+y*y+z*z;
	}
	// ��_�ԋ���
	static float distanceSq( const Vec3& lhs, const Vec3& rhs )
	{
		const float dx = lhs.x - rhs.x;
		const float dy = lhs.y - rhs.y;
		const float dz = lhs.z - rhs.z;
		return dx*dx+dy*dy+dz*dz;
	}
	// ����
	static float dot( const Vec3& lhs, const Vec3& rhs )
	{
		return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
	}

public:
	float x;
	float y;
	float z;
};

// �O�p�̒��ł̏d�S���W�����߂�
void calcBarycentricTriangle( 
	const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& p,
	float& fa, float& fb, float& fc )
{
	const Vec2 v0 = b - a;
	const Vec2 v1 = c - a;
	const Vec2 v2 = p - a;
	// ���ς��v�Z���Ă���
	const float dot00 = Vec2::dot(v0,v0);
	const float dot01 = Vec2::dot(v0,v1);
	const float dot02 = Vec2::dot(v0,v2);
	const float dot10 = dot01;
	const float dot11 = Vec2::dot(v1,v1);
	const float dot12 = Vec2::dot(v1,v2);
	const float dot20 = dot02;
	const float dot21 = dot12;
	const float dot22 = Vec2::dot(v2,v2);
	/*
	det  = |v0*v0 v1*v0|
	       |v0*v1 v1*v1|
	detB = |v2*v0 v1*v0|
	       |v2*v1 v1*v1|
	detC = |v0*v0 v2*v0|
	       |v0*v1 v2*v1|
	*/
	const float det  = dot00*dot11 - dot10*dot01;
	const float detB = dot20*dot11 - dot10*dot21;
	const float detC = dot00*dot21 - dot20*dot01;
	// �e�d�S���W�����߂�
	fb = detB/det;
	fc = detC/det;
	fa = 1.0f - (fb + fc);
}

// main�֐�
void main()
{
	{ // �񎟌��ł̎O�p�`�ł̏d�S���W
		const Vec2 a(1.0f,1.0f);
		const Vec2 b(2.0f,1.0f);
		const Vec2 c(1.0f,2.0f);
		const Vec2 p(1.5f,1.5f);
		float fa,fb,fc;
		calcBarycentricTriangle(a,b,c,p,fa,fb,fc);
		printf("a:%f b:%f c:%f\n", fa, fb, fc );
	}
	{ // �O�����ł̎O�p���ł̏d�S���W
		// TODO ����
	}
}
