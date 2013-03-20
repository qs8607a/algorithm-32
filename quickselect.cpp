/*
quickSelect
- 中央値を探索する
- 平均的にO(n)
- ※バグもち。そのうち直す。
*/

#include <cstdio>
#include <algorithm>
#include <functional>
#include <vector>

// 中央値を返す
int quickSelect( int* elements, int left, int right, int selectNo )
{
	// 要素が1つしかない場合
	if( left == right )
	{
		return elements[left];
	}
	// 要素が二つしかない場合
	else if( right - left == 1 )
	{
		if( elements[left] > elements[right] )
		{ std::swap(elements[left],elements[right]); }
		if( left == selectNo )
		{ return elements[left]; }
		else 
		{ return elements[right]; }
	}

	// pivotの決定を行う
	int numSameNumber = left;
	while(
		numSameNumber < right &&
		elements[numSameNumber] == elements[numSameNumber+1] )
	{
		++numSameNumber;
	}
	// 同値のみの列なので先頭要素をそのまま返す
	if( (numSameNumber != left) && numSameNumber + 1 == (right - left + 1) )
	{
		return elements[0];
	}
	// 軸要素の決定(隣あう値が違う値で、その高いほうであれば列中の最低値では確実にない)し、最後尾に移動
	if( elements[numSameNumber] < elements[numSameNumber+1] )
	{ std::swap( elements[numSameNumber+1], elements[right] ); }
	else
	{ std::swap( elements[numSameNumber], elements[right] ); }
	int pivotValue = elements[right];
	// 二つの要素に分割
	int lowArrayIndex = left;
	for(int i=left;i<right;++i)
	{
		if( elements[i] < pivotValue )
		{
			std::swap( elements[lowArrayIndex], elements[i] );
			++lowArrayIndex;
		}
	}
	// 後半の列の先頭とpivot値のある最後尾を交換
	std::swap( elements[lowArrayIndex], elements[right] );
	// 分割の中心が中心より右だったら左の中でさらに選択動作
	if( selectNo < lowArrayIndex )
	{
		return quickSelect( elements, left, lowArrayIndex, selectNo );
	}
	// 分割の中心が中心より左だったら右の中でさらに選択動作
	else if( lowArrayIndex < selectNo )
	{
		return quickSelect( elements, lowArrayIndex, right, selectNo );
	}
	else
	{
		return elements[lowArrayIndex];
	}
}

// quickSelect
int quickSelect( int* elements, int numElements, int selectNo )
{
	return quickSelect( elements, 0, numElements-1, selectNo );
}

// 全体をソートしてから選択
int bruteSelect( int* elements, int numElements, int selectNo )
{
	// 全体をソートして指定した場所を返す
	std::sort( elements, elements + numElements );
	return elements[selectNo];
}

// main関数
void main()
{
	for( int testNo=0;testNo<100;++testNo)
	{
		std::vector<int> testValue;
		testValue.resize( rand() % 100 + 1 );
		for( int& i : testValue ){ i = rand(); }
		const int selectPos = rand() % testValue.size();
		std::vector<int> copy0 = testValue;
		std::vector<int> copy1 = testValue;
		int selectedNo0 = quickSelect( &copy0[0], copy0.size(), selectPos );
		int selectedNo1 = bruteSelect( &copy1[0], copy1.size(), selectPos );
		if( selectedNo0 != selectedNo1 )
		{
			printf("%d\n",testNo);
			printf("testNo%d\n select:%d\n %d-%d\n", testNo, selectPos,  selectedNo0, selectedNo1 );
			printf(" numElem:%d\n ", testValue.size() );
			printf(" ");
			for( int i:testValue){ printf("%d ", i); }
			printf("\n");
		}
	}
}
