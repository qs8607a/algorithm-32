/**
kd-tree
- 最近傍探索に使う
- 凄い次元が上がると遅いらしい
参考: 
  http://ja.wikipedia.org/wiki/Kd%E6%9C%A8
  http://d.hatena.ne.jp/higepon/20101024/1287914277
  http://d.hatena.ne.jp/higepon/20101026/1288095914
  http://stackoverflow.com/questions/1402014/kdtree-implementation-c (張ってあるソース盛大にバグってるので注意)
*/

#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <vector>
#include <Windows.h>
#pragma comment(lib,"winmm.lib")

#define CHECK(expr) if(!(expr)){ __asm{ int 3} }
#define INLINE __forceinline

// 三次元ベクトル
struct Vec3
{
public:
	// コンストラクタ
	INLINE Vec3()
		:x(0.0f),y(0.0f),z(0.0f)
	{}
	INLINE Vec3( float ax, float ay, float az )
		:x(ax),y(ay),z(az)
	{}
	// 長さ二乗
	INLINE float lengthSq() const
	{
		return x * x + y * y + z * z;
	}
	// 二点間距離二乗
	INLINE static float DistanceSq( const Vec3& lhs, const Vec3& rhs )
	{
		const float dx = lhs.x - rhs.x;
		const float dy = lhs.y - rhs.y;
		const float dz = lhs.z - rhs.z;
		return dx*dx + dy*dy + dz*dz;
	}
	// operator
	INLINE const float& operator[](int index) const { return *(&x+index); }
	INLINE float& operator[](int index) { return *(&x+index); }
	INLINE bool operator != ( const Vec3& other ) const
	{
		return 
			x != other.x ||
			y != other.y ||
			z != other.z;
	}
	INLINE Vec3 operator - ( const Vec3& other )
	{
		return Vec3( x-other.x, y-other.y, z-other.z );
	}
public:
	float x;
	float y;
	float z;
};

// 最近傍情報
struct NearestInfo
{
	// 近接座標
	Vec3 point;
	// 近接座標までの距離の二乗
	float distSq;
	// operator
	INLINE bool operator != ( const NearestInfo& other ) const
	{
		return point != other.point || distSq != other.distSq;
	}
};

// kd-treeのノード
class kdTreeNode
{
public:
	// コンストラクタ。引数を破壊する点に注意。
	kdTreeNode::kdTreeNode()
		:left(NULL),
		right(NULL)
	{
	}
	// デストラクタ
	~kdTreeNode()
	{
		//delete left;
		//delete right;
	}
	// コンストラクタ。引数を破壊する点に注意。
	void kdTreeNode::create(Vec3* pointList, int pointLength, int depth )
	{
		// 葉になることが、すでに確定している場合
		if( pointLength == 1 )
		{
			left	= NULL;
			right	= NULL;
			point   = *pointList;
			return;
		}

		// ソート用述語
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
		一要素の場合、0|1|0
		二要素の場合、1|1|0
		三要素の場合、1|1|1
		四要素の場合、2|1|1
		五要素の場合、2|1|2
		*/
		const int beforeLength = pointLength / 2;
		const int afterLength  = ((pointLength+1) / 2) - 1;
		point = pointList[beforeLength];
		CHECK( beforeLength!=0 );
		left = new kdTreeNode();
		left->create( pointList, beforeLength, depth + 1 );
		if( afterLength != 0 )
		{
			right = new kdTreeNode();
			right->create( pointList + beforeLength + 1, afterLength, depth + 1 );
		}
	}
	// ブルートフォースで最近接を探索。デバッグ用。
	INLINE NearestInfo findNearestBrute( const Vec3& targetPoint )
	{
		NearestInfo ni;
		ni.distSq = Vec3::DistanceSq( targetPoint, point );
		ni.point = point;

		if( left )
		{
			NearestInfo niLeft = left->findNearestBrute(targetPoint);
			if( niLeft.distSq < ni.distSq )
			{
				ni = niLeft;
			}
		}
		if( right )
		{
			NearestInfo niRight = right->findNearestBrute(targetPoint);
			if( niRight.distSq < ni.distSq )
			{
				ni = niRight;
			}
		}
		return ni;
	}
	// 最近傍点を取得
	INLINE NearestInfo findNearest(const Vec3& targetPoint )
	{
		return findNearest( targetPoint, 0 );
	}
private:
	// 最近接を取得
	INLINE NearestInfo findNearest(const Vec3& targetPoint, int depth )
	{
		if( !right )
		{
			NearestInfo ci;
			ci.distSq = Vec3::DistanceSq( point, targetPoint );
			ci.point = point;
			// 両葉がない場合
			if(!left)
			{
				return ci;
			}
			// 左だけの場合
			else
			{
				const float d1 = Vec3::DistanceSq(left->point, targetPoint);
				if( ci.distSq < d1 )
				{
					return ci;
				}
				else
				{
					NearestInfo ci0;
					ci0.distSq = d1;
					ci0.point = left->point;
					return ci0;
				}
			}
		}
		// 現在の軸方向距離
		const float axisDist = point[depth] - targetPoint[depth];
		// 現在居る側を出す
		const int side = axisDist < 0.0f;
		// 軸交換
		depth = (depth + 1) % 3;
		// 現在居る領域の最近接
		NearestInfo nearest0 = childlen[side]->findNearest(targetPoint, depth);
		// "仕切り"と"対岸"の可能性がなくなったので、現在の領域の最近接を採用
		if( nearest0.distSq < axisDist * axisDist  )
		{ return nearest0; }
		// "仕切り"
		NearestInfo nearest1;
		nearest1.distSq = Vec3::DistanceSq(point,targetPoint);
		nearest1.point = point;
		// "対岸"
		NearestInfo nearest2 = childlen[side^1]->findNearest(targetPoint, depth);
		// contact0/1/2で一番近いものを採用する
		if( nearest0.distSq < nearest1.distSq && nearest0.distSq < nearest2.distSq )
		{ return nearest0; }
		else if( nearest1.distSq < nearest2.distSq )
		{ return nearest1; }
		else
		{ return nearest2; }
	}
private:
	// このノードの位置
	Vec3 point;
	// 子ノード
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

// メイン関数
void main()
{
	std::vector<Vec3> points;
	points.resize(1000);
	for( Vec3& p : points )
	{
		p = Vec3( rf(), rf(), rf() );
	}
	kdTreeNode kdtree;
	kdtree.create(&points[0],points.size(),0 );
	
#if 1 // 動作テスト
	const int NUM_TEST = 1000;
	for( unsigned int i=0;i<NUM_TEST;++i)
	{
		const Vec3 testPoint( rf(), rf(), rf() );
		const NearestInfo result0 = kdtree.findNearest( testPoint );
		const NearestInfo result1 = kdtree.findNearestBrute( testPoint );
		CHECK( !(result0!=result1) );
		const Vec3& p0 = result0.point;
		const Vec3& p1 = result1.point;
		printf("%d\n",i);
		printf( "  %f %f\n", testPoint.x, testPoint.y );
		printf( "  %f %f(%f)\n", p0.x, p0.y, result0.distSq );
		printf( "  %f %f(%f)\n", p1.x, p1.y, result1.distSq );
	}
#endif
#if 0 // 性能テスト
	const int NUM_TEST = 96 * 16 * 100;
	std::vector<Vec3> tests;
	for( unsigned int i=0;i<NUM_TEST;++i)
	{
		tests.push_back( Vec3( rf(), rf(), rf() ) );
	}
	const long start = timeGetTime();
	for( unsigned int i=0;i<NUM_TEST;++i)
	{
		const NearestInfo result0 = kdtree.findNearest( tests[i] );
		result0;
	}
	const long elase = timeGetTime() - start;
	printf("%dms\n",elase);
#endif
}
