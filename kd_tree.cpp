/**
kd-tree
- 空間にある点データの最近傍探索、N個の近傍探索、範囲内の列挙が行える
- 空間的に偏りのあるデータを効率良く格納できる
- 構築 O(nlog(n)), 探索O(log(n))
- 凄い次元が上がると遅いらしい
- 大抵 x->y->z->x...と更新しているが、この順番は自由にしていい。zにほとんど散らばっていなければ x->y->x...でも良い。
参考: 
  http://ja.wikipedia.org/wiki/Kd%E6%9C%A8
  http://d.hatena.ne.jp/higepon/20101024/1287914277
  http://d.hatena.ne.jp/higepon/20101026/1288095914
  http://stackoverflow.com/questions/1402014/kdtree-implementation-c (参考元ではあるが、張ってあるソースが盛大にバグってるので注意)
*/

#include <cstdlib>
#include <algorithm>
#include <vector>
#include <array>
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

// kd-treeの探索結果情報
struct kdTreeFindInfo
{
	// 近傍座標
	Vec3 point;
	// 近傍座標までの距離の二乗
	float distSq;
	// operator
	INLINE bool operator != ( const kdTreeFindInfo& other ) const
	{
		return point != other.point || distSq != other.distSq;
	}
};

// kd-treeのノード
class kdTreeNode
{
public:
	// コンストラクタ。引数を破壊する事に注意
	INLINE kdTreeNode(Vec3* pointList, int pointLength, int depth )
		:left(NULL),
		right(NULL)
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
		left = new kdTreeNode( pointList, beforeLength, depth + 1 );
		if( afterLength != 0 )
		{
			right = new kdTreeNode( pointList + beforeLength + 1, afterLength, depth + 1 );
		}
	}
	// デストラクタ
	INLINE ~kdTreeNode()
	{
		delete left;
		delete right;
	}
	// 最近傍を総当りで探索
	INLINE kdTreeFindInfo findNearestBrute( const Vec3& targetPoint ) const
	{
		kdTreeFindInfo fi;
		fi.distSq = Vec3::DistanceSq( targetPoint, point );
		fi.point = point;

		if( left )
		{
			kdTreeFindInfo fiLeft = left->findNearestBrute(targetPoint);
			if( fiLeft.distSq < fi.distSq )
			{
				fi = fiLeft;
			}
		}
		if( right )
		{
			kdTreeFindInfo fiRight = right->findNearestBrute(targetPoint);
			if( fiRight.distSq < fi.distSq )
			{
				fi = fiRight;
			}
		}
		return fi;
	}
	// 最近傍点を探索
	INLINE kdTreeFindInfo findNearest(const Vec3& targetPoint ) const
	{
		return findNearest( targetPoint, 0 );
	}
	// 近傍4点を探索(N近傍のサンプル。テンプレート使ってN近傍に対応させることもできる)
	INLINE void findNearest4(const Vec3& targetPoint, std::array<kdTreeFindInfo,4>& findInfos ) const
	{
		findInfos[0].distSq = FLT_MAX;
		findInfos[1].distSq = FLT_MAX;
		findInfos[2].distSq = FLT_MAX;
		findInfos[3].distSq = FLT_MAX;
		const int depth = 0;
		findNearest4( targetPoint, findInfos , depth );
	}
	// 近傍4点を総当りで探索
	INLINE void findNearest4Brute(const Vec3& targetPoint, std::array<kdTreeFindInfo,4>& findInfos ) const
	{
		findInfos[0].distSq = FLT_MAX;
		findInfos[1].distSq = FLT_MAX;
		findInfos[2].distSq = FLT_MAX;
		findInfos[3].distSq = FLT_MAX;
		const int depth = 0;
		findNearest4Brute( targetPoint, findInfos, depth );
	}
private:
	// 最近傍を取得
	INLINE kdTreeFindInfo findNearest(const Vec3& targetPoint, int depth ) const
	{
		if( !right )
		{
			kdTreeFindInfo fiHere;
			fiHere.distSq = Vec3::DistanceSq( point, targetPoint );
			fiHere.point = point;
			// 両葉がない場合
			if(!left)
			{
				return fiHere;
			}
			// 左だけの場合
			else
			{
				kdTreeFindInfo fiLeft;
				fiLeft.distSq = Vec3::DistanceSq(left->point, targetPoint);
				fiLeft.point = left->point;
				return fiHere.distSq < fiLeft.distSq ? fiHere : fiLeft;
			}
		}
		// 現在の軸方向距離
		const float axisDist = point[depth] - targetPoint[depth];
		// 現在居る側を出す
		const int side = axisDist < 0.0f;
		// 軸交換
		depth = (depth + 1) % 3;
		// 現在居る領域の最近傍
		const kdTreeFindInfo fi0 = childlen[side]->findNearest(targetPoint, depth);
		// "仕切り"と"対岸"の可能性がなくなったので、現在の領域の最近傍を採用
		if( fi0.distSq < axisDist * axisDist  )
		{ return fi0; }
		// "仕切り"
		kdTreeFindInfo fi1;
		fi1.distSq = Vec3::DistanceSq(point,targetPoint);
		fi1.point = point;
		// "対岸"
		const kdTreeFindInfo fi2 = childlen[side^1]->findNearest(targetPoint, depth);
		// contact0/1/2で一番近いものを採用する
		if( fi0.distSq < fi1.distSq && fi0.distSq < fi2.distSq )
		{ return fi0; }
		else if( fi1.distSq < fi2.distSq )
		{ return fi1; }
		else
		{ return fi2; }
	}
	// 近傍4点を取得
	INLINE void findNearest4(const Vec3& targetPoint,  std::array<kdTreeFindInfo,4>& findInfos, int depth ) const
	{
		// "仕切り"を格納
		kdTreeFindInfo fiHere;
		fiHere.distSq = Vec3::DistanceSq( point, targetPoint );
		fiHere.point = point;
		tryPush( findInfos, fiHere );
		// 右がない場合
		if( !right )
		{
			// 両葉がない場合
			if(!left)
			{		
				return;
			}
			// 左だけの場合
			else
			{
				kdTreeFindInfo ni0;
				ni0.distSq = Vec3::DistanceSq(left->point, targetPoint);
				ni0.point = left->point;
				tryPush( findInfos, ni0 );
				return ;
			}
		}
		// 現在の軸方向距離
		const float axisDist = point[depth] - targetPoint[depth];
		// 現在居る側を出す
		const int side = axisDist < 0.0f;
		// 軸交換
		depth = (depth + 1) % 3;
		// 現在居る領域の最近傍
		childlen[side]->findNearest4(targetPoint, findInfos, depth);
		// "仕切り"と"対岸"の可能性がなくなったので、現在の領域の最近傍を採用
		if( farDistSq(findInfos) < axisDist * axisDist  )
		{ 
			return;
		}
		// "対岸"
		childlen[side^1]->findNearest4(targetPoint, findInfos, depth);
	}
	// 近傍4点を総当りで探索
	INLINE void findNearest4Brute(const Vec3& targetPoint, std::array<kdTreeFindInfo,4>& findInfos, int depth ) const
	{
		kdTreeFindInfo fi;
		fi.distSq = Vec3::DistanceSq( targetPoint, point );
		fi.point  = point;
		tryPush( findInfos, fi );
		if( left ) { left->findNearest4Brute( targetPoint, findInfos, depth + 1 ); }
		if( right ) { right->findNearest4Brute( targetPoint, findInfos, depth + 1 ); }
	}
	// 近傍点候補の中への追加を試みる
	INLINE void tryPush( std::array<kdTreeFindInfo,4>& findInfos, const kdTreeFindInfo& findInfo ) const
	{
		// 常に最後尾に一番遠い要素を配置
		if( findInfo.distSq < farDistSq( findInfos ) )
		{
			// 現在最後尾以外で、一番遠い要素のインデックスを算出
			float farDist = findInfo.distSq;
			int farIndex = 3;
			for( int index=0;index<3;++index)
			{
				if( findInfos[index].distSq > farDist )
				{
					farDist = findInfos[index].distSq;
					farIndex = index;
				}
			}
			// 最後尾に一番遠い要素を配置し、空いた場所に新しい要素を入れる
			findInfos[3] = findInfos[farIndex];
			findInfos[farIndex] = findInfo;
		}
	}
	// 近傍点候補の中で一番遠い座標までの距離の二乗
	INLINE float farDistSq( std::array<kdTreeFindInfo,4>& findInfos ) const
	{
		// 常に最後尾に一番遠い座標が格納されている
		return findInfos[3].distSq;
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
// [-1.0,1.0)
float rf()
{
	return (float)((double)rand()/(double)RAND_MAX)*2.0f - 1.0f;
}
// メイン関数
void main()
{
	std::vector<Vec3> points;
	points.resize(10000);
	for( Vec3& p : points )
	{
		p = Vec3( rf(), rf(), rf() );
	}
	kdTreeNode kdtree(&points[0],points.size(),0 );
	
#if 0 // 動作テスト(最近傍)
	{
		for( unsigned int i=0;i<1000;++i)
		{
			const Vec3 testPoint( rf(), rf(), rf() );
			const kdTreeFindInfo result0 = kdtree.findNearest( testPoint );
			const kdTreeFindInfo result1 = kdtree.findNearestBrute( testPoint );
			CHECK( !(result0!=result1) );
			const Vec3& p0 = result0.point;
			const Vec3& p1 = result1.point;
			printf("%d\n",i);
			printf( "  %f %f\n", testPoint.x, testPoint.y );
			printf( "  %f %f(%f)\n", p0.x, p0.y, result0.distSq );
			printf( "  %f %f(%f)\n", p1.x, p1.y, result1.distSq );
		}
	}
#endif
#if 1 // 性能テスト(最近傍)
	{
		const int NUM_TEST = 10000;
		std::vector<Vec3> tests;
		for( unsigned int i=0;i<NUM_TEST;++i)
		{
			tests.push_back( Vec3( rf(), rf(), rf() ) );
		}
		long start = timeGetTime();
		for( unsigned int i=0;i<NUM_TEST;++i)
		{
			const kdTreeFindInfo result = kdtree.findNearest( tests[i] );
			result;
		}
		long elapse = timeGetTime() - start;
		printf("kd-tree:%dms\n",elapse);

		start = timeGetTime();
		for( unsigned int i=0;i<NUM_TEST;++i)
		{
			const kdTreeFindInfo result = kdtree.findNearestBrute( tests[i] );
			result;
		}
		elapse = timeGetTime() - start;
		printf("Brute: %dms\n",elapse);
	}
#endif

#if 0 // 動作テスト(近傍N点)
	for( unsigned int i=0;i<1000;++i)
	{
		const Vec3 testPoint( rf(), rf(), rf() );
		std::array<kdTreeFindInfo,4> resultsBrute;
		std::array<kdTreeFindInfo,4> results;
		kdtree.findNearest4( testPoint, results );
 		kdtree.findNearest4Brute( testPoint, resultsBrute );
		std::sort( results.begin(), results.end(), [](kdTreeFindInfo&x,kdTreeFindInfo&y)->bool { return x.distSq < y.distSq; } );
		std::sort( resultsBrute.begin(), resultsBrute.end(), [](kdTreeFindInfo&x,kdTreeFindInfo&y)->bool { return x.distSq < y.distSq; } );
		printf("%d\n",i);
		printf( "  %f %f %f\n", testPoint.x, testPoint.y, testPoint.z );
		CHECK( results.size() == resultsBrute.size() );
		printf( "  kd-tree                     brute\n" );
		for( unsigned int j=0;j<results.size();++j)
		{
			kdTreeFindInfo& ni0 = results[j];
			kdTreeFindInfo& ni1 = resultsBrute[j];
			CHECK( !(ni0!=ni1) );
			Vec3& p0 = ni0.point;
			Vec3& p1 = ni1.point;
			printf( "  %2.2f %2.2f %2.2f(%f)--", p0.x, p0.y, p0.z, ni0.distSq );
			printf( "  %2.2f %2.2f %2.2f(%f)\n", p1.x, p1.y, p1.z, ni1.distSq );
		}
	}
#endif

#if 1 // 性能テスト(近傍N点)
	{
		const int NUM_TEST = 10000;
		std::vector<Vec3> tests;
		for( unsigned int i=0;i<NUM_TEST;++i)
		{
			tests.push_back( Vec3( rf(), rf(), rf() ) );
		}
		std::array<kdTreeFindInfo,4> results;
		long start = timeGetTime();
		for( unsigned int i=0;i<NUM_TEST;++i)
		{
			kdtree.findNearest4( tests[i], results );
			results;
		}
		long elapse = timeGetTime() - start;
		printf("kd-tree:%dms\n",elapse);

		start = timeGetTime();
		for( unsigned int i=0;i<NUM_TEST;++i)
		{
			kdtree.findNearest4Brute( tests[i], results );
			results;
		}
		elapse = timeGetTime() - start;
		printf("Brute:%dms\n",elapse);
	}
#endif
}
