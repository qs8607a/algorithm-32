/*
quicksort
- http://ja.wikipedia.org/wiki/%E3%82%AF%E3%82%A4%E3%83%83%E3%82%AF%E3%82%BD%E3%83%BC%E3%83%88
- http://stricter.org/docs/tech/quicksort/
*/

#include <cstdio>
#include <algorithm>

// quick sortを行う。引数を破壊することに注意
template<typename T,typename Pred>
static void quickSort( T* elements, size_t numElements, Pred pred )
{
	// 要素が一つ以下の場合はソートする必要がないので何もしない
	if( numElements < 2 )
	{ return ; }
	/*
	pivotの決定を行う。
	「先頭の値を使う」「中央にある値を使う」「先頭と二つ目の値を比べ大きいほうを使う」
	などの素朴な方法が例として上がることが多いが、全て同値の列が来た場合に無限再帰になってしまう。
	「全て同値になってしまっているか」のチェックと「ピボットは少なくとも最低値ではない」のチェックがどうしても必要。
	*/
	int numSameNumber = 0;
	while(
		!pred( elements[numSameNumber],elements[numSameNumber+1] ) && 
		!pred( elements[numSameNumber+1],elements[numSameNumber] ) &&
		numSameNumber < numElements )
	{
		++numSameNumber;
	}
	// 同値のみの列なのでソートの必要なし
	if( numSameNumber + 1 == numElements )
	{
		return ;
	}
	// 軸要素の決定(隣あう値が違う値で、その高いほうであれば列中の最低値では確実にない)し、最後尾に移動
	if( elements[numSameNumber] < elements[numSameNumber+1] )
	{ std::swap( elements[numSameNumber+1], elements[numElements-1] ); }
	else
	{ std::swap( elements[numSameNumber], elements[numElements-1] ); }
	T pivotValue = elements[numElements-1];
	// 二つの要素に分割
	int lowArrayIndex = 0;
	for(int i=0;i<numElements;++i)
	{
		if( pred( elements[i], pivotValue ) )
		{
			std::swap( elements[lowArrayIndex], elements[i] );
			++lowArrayIndex;
		}
	}
	// 分割したデータ列をさらにソート
	quickSort( elements, lowArrayIndex, pred );
	quickSort( elements + lowArrayIndex, numElements - lowArrayIndex, pred );
}

// main関数
void main()
{
	int elements[] = {8,2,9,2,3,9};
	quickSort( elements, sizeof(elements)/sizeof(*elements), [](int x,int y){ return x<y; } );
}
