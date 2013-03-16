/**
kd-tree
- �ŋߖT�T���Ɏg��
- �����������オ��ƒx���炵��
�Q�l: 
  http://ja.wikipedia.org/wiki/Kd%E6%9C%A8
  http://d.hatena.ne.jp/higepon/20101024/1287914277
  http://d.hatena.ne.jp/higepon/20101026/1288095914
  http://stackoverflow.com/questions/1402014/kdtree-implementation-c (�����Ă���\�[�X�s��Ƀo�O���Ă钍��)
*/

#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <vector>

#define CHECK(expr) if(!(expr)){ __asm{ int 3} }

// �O�����x�N�g��
struct Vec3
{
public:
	// �R���X�g���N�^
	Vec3()
		:x(0.0f),y(0.0f),z(0.0f)
	{}
	Vec3( float ax, float ay, float az )
		:x(ax),y(ay),z(az)
	{}
	// �������
	float lengthSq() const
	{
		return x * x + y * y + z * z;
	}
	// ��_�ԋ������
	static float DistanceSq( const Vec3& lhs, const Vec3& rhs )
	{
		const float dx = lhs.x - rhs.x;
		const float dy = lhs.y - rhs.y;
		const float dz = lhs.z - rhs.z;
		return dx*dx + dy*dy + dz*dz;
	}
	// operator
	const float& operator[](int index) const { return *(&x+index); }
	float& operator[](int index) { return *(&x+index); }
	bool operator != ( const Vec3& other ) const
	{
		return 
			x != other.x ||
			y != other.y ||
			z != other.z;
	}
	Vec3 operator - ( const Vec3& other )
	{
		return Vec3( x-other.x, y-other.y, z-other.z );
	}
public:
	float x;
	float y;
	float z;
};

// �ŋߖT���
struct NearestInfo
{
	// �ߐڍ��W
	Vec3 point;
	// �ߐڍ��W�܂ł̋���
	float dist;
	// operator
	bool operator != ( const NearestInfo& other ) const
	{
		return point != other.point || dist != other.dist;
	}
};

// kd-tree�̃m�[�h
class kdTreeNode
{
public:
	// �R���X�g���N�^�B������j�󂷂�_�ɒ��ӁB
	kdTreeNode::kdTreeNode(Vec3* pointList, int pointLength, int depth = 0 )
		:left(NULL),
		right(NULL)
	{
		// �t�ɂȂ邱�Ƃ��A���łɊm�肵�Ă���ꍇ
		if( pointLength == 1 )
		{
			left	= NULL;
			right	= NULL;
			point   = *pointList;
			return;
		}

		/* 
		HACK
		�����l�I�����\�[�g���Ă������Ă�̂̓}�Y���B
		�I���A���S���Y���ł����ƒ����l���擾����悤�ɂ���B
		*/
		// �\�[�g�p�q��
		class Local
		{
		public:
			static int CompareFloat(float lhs, float rhs)
			{
				if( lhs < rhs ){ return -1; }
				else if( lhs > rhs ){ return 1; }
				else{ return 0; }
			}
			static int compareVec3X(const void* lhs, const void* rhs){ return CompareFloat(((Vec3*)lhs)->x,((Vec3*)rhs)->x); }
			static int compareVec3Y(const void* lhs, const void* rhs){ return CompareFloat(((Vec3*)lhs)->y,((Vec3*)rhs)->y); }
			static int compareVec3Z(const void* lhs, const void* rhs){ return CompareFloat(((Vec3*)lhs)->z,((Vec3*)rhs)->z); }
		};
		switch( depth % 3 )
		{
		case 0: qsort(pointList, pointLength, sizeof(Vec3), Local::compareVec3X); break;
		case 1: qsort(pointList, pointLength, sizeof(Vec3), Local::compareVec3Y); break;
		case 2: qsort(pointList, pointLength, sizeof(Vec3), Local::compareVec3Z); break;
		}

		/*
		��v�f�̏ꍇ�A0|1|0
		��v�f�̏ꍇ�A1|1|0
		�O�v�f�̏ꍇ�A1|1|1
		�l�v�f�̏ꍇ�A2|1|1
		�ܗv�f�̏ꍇ�A2|1|2
		*/
		const int beforeLength = pointLength / 2;
		const int afterLength  = ((pointLength+1) / 2) - 1;
		point = pointList[beforeLength];
		CHECK( beforeLength!=0 );
		left = new kdTreeNode( pointList, beforeLength, depth + 1);
		if( afterLength != 0 )
		{
			right = new kdTreeNode( pointList + beforeLength + 1, afterLength, depth + 1);
		}
	}
	// �f�X�g���N�^
	~kdTreeNode()
	{
		delete left;
		delete right;
	}
	// �u���[�g�t�H�[�X�ōŋߐڂ�T���B�f�o�b�O�p�B
	NearestInfo findNearestBrute( const Vec3& targetPoint )
	{
		NearestInfo ni;
		ni.dist = Vec3::DistanceSq( targetPoint, point );
		ni.point = point;

		if( left )
		{
			NearestInfo niLeft = left->findNearestBrute(targetPoint);
			if( niLeft.dist < ni.dist )
			{
				ni = niLeft;
			}
		}
		if( right )
		{
			NearestInfo niRight = right->findNearestBrute(targetPoint);
			if( niRight.dist < ni.dist )
			{
				ni = niRight;
			}
		}
		return ni;
	}
	// �ŋߖT�_���擾
	NearestInfo findNearest(const Vec3& targetPoint )
	{
		return findNearest( targetPoint, 0 );
	}
private:
	// �ŋߐڂ��擾
	NearestInfo kdTreeNode::findNearest(const Vec3& targetPoint, int depth )
	{
		if( !right )
		{
			const Vec3 r = point - targetPoint;
			NearestInfo ci;
			ci.dist = r.lengthSq();
			ci.point = point;
			// ���t���Ȃ��ꍇ
			if(!left)
			{
				return ci;
			}
			// �������̏ꍇ
			else
			{
				Vec3 r = left->point - targetPoint;
				const float d1 = r.lengthSq();
				if( ci.dist < d1 )
				{
					return ci;
				}
				else
				{
					NearestInfo ci0;
					ci0.dist = d1;
					ci0.point = left->point;
					return ci0;
				}
			}
		}
		// ���݂̎���������
		const float axisDist = point[depth] - targetPoint[depth];
		// ���݋��鑤���o��
		const int side = axisDist < 0.0f;
		// ������
		depth = (depth + 1) % 3;
		// ���݋���̈�̍ŋߐ�
		NearestInfo nearest0 = childlen[side]->findNearest(targetPoint, depth);
		// "�d�؂�"��"�Ί�"�̉\�����Ȃ��Ȃ����̂ŁA���݂̗̈�̍ŋߐڂ��̗p
		if( nearest0.dist < axisDist * axisDist  )
		{ return nearest0; }
		// "�d�؂�"
		NearestInfo nearest1;
		nearest1.dist = Vec3::DistanceSq(point,targetPoint);
		nearest1.point = point;
		// "�Ί�"
		NearestInfo nearest2 = childlen[side^1]->findNearest(targetPoint, depth);
		// contact0/1/2�ň�ԋ߂����̂��̗p����
		if( nearest0.dist < nearest1.dist && nearest0.dist < nearest2.dist )
		{ return nearest0; }
		else if( nearest1.dist < nearest2.dist )
		{ return nearest1; }
		else
		{ return nearest2; }
	}
private:
	// ���̃m�[�h�̈ʒu
	Vec3 point;
	// �q�m�[�h
	union
	{
		struct
		{
			kdTreeNode* left;
			kdTreeNode* right;
		};
		kdTreeNode* childlen[2];
	};
};

// [0.0,1.0)
float rf()
{
	return (float)((double)rand()/(double)RAND_MAX);
}

// ���C���֐�
void main()
{
	std::vector<Vec3> points;
	points.resize(500);
	for( Vec3& p : points )
	{
		p = Vec3( rf(), rf(), rf() );
	}
	kdTreeNode node2d(&points[0],points.size());
	//�e�X�g�|�C���g
	const int NUM_TEST = 1000;
	for( unsigned int i=0;i<NUM_TEST;++i)
	{
		const Vec3 testPoint( rf(), rf(), rf() );
		const NearestInfo result0 = node2d.findNearest( testPoint );
		const NearestInfo result1 = node2d.findNearestBrute( testPoint );
		CHECK( !(result0!=result1) );
		const Vec3& p0 = result0.point;
		const Vec3& p1 = result1.point;
		printf("%d\n",i);
		printf( "  %f %f\n", testPoint.x, testPoint.y );
		printf( "  %f %f(%f)\n", p0.x, p0.y, result0.dist );
		printf( "  %f %f(%f)\n", p1.x, p1.y, result1.dist );
	}
}
