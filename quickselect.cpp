/*
quickSelect
- �����l��T������
- ���ϓI��O(n)
- ���o�O�����B���̂��������B
*/

#include <cstdio>
#include <algorithm>
#include <functional>
#include <vector>

// �����l��Ԃ�
int quickSelect( int* elements, int left, int right, int selectNo )
{
	// �v�f��1�����Ȃ��ꍇ
	if( left == right )
	{
		return elements[left];
	}
	// �v�f��������Ȃ��ꍇ
	else if( right - left == 1 )
	{
		if( elements[left] > elements[right] )
		{ std::swap(elements[left],elements[right]); }
		if( left == selectNo )
		{ return elements[left]; }
		else 
		{ return elements[right]; }
	}

	// pivot�̌�����s��
	int numSameNumber = left;
	while(
		numSameNumber < right &&
		elements[numSameNumber] == elements[numSameNumber+1] )
	{
		++numSameNumber;
	}
	// ���l�݂̗̂�Ȃ̂Ő擪�v�f�����̂܂ܕԂ�
	if( (numSameNumber != left) && numSameNumber + 1 == (right - left + 1) )
	{
		return elements[0];
	}
	// ���v�f�̌���(�ׂ����l���Ⴄ�l�ŁA���̍����ق��ł���Η񒆂̍Œ�l�ł͊m���ɂȂ�)���A�Ō���Ɉړ�
	if( elements[numSameNumber] < elements[numSameNumber+1] )
	{ std::swap( elements[numSameNumber+1], elements[right] ); }
	else
	{ std::swap( elements[numSameNumber], elements[right] ); }
	int pivotValue = elements[right];
	// ��̗v�f�ɕ���
	int lowArrayIndex = left;
	for(int i=left;i<right;++i)
	{
		if( elements[i] < pivotValue )
		{
			std::swap( elements[lowArrayIndex], elements[i] );
			++lowArrayIndex;
		}
	}
	// �㔼�̗�̐擪��pivot�l�̂���Ō��������
	std::swap( elements[lowArrayIndex], elements[right] );
	// �����̒��S�����S���E�������獶�̒��ł���ɑI�𓮍�
	if( selectNo < lowArrayIndex )
	{
		return quickSelect( elements, left, lowArrayIndex, selectNo );
	}
	// �����̒��S�����S��荶��������E�̒��ł���ɑI�𓮍�
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

// �S�̂��\�[�g���Ă���I��
int bruteSelect( int* elements, int numElements, int selectNo )
{
	// �S�̂��\�[�g���Ďw�肵���ꏊ��Ԃ�
	std::sort( elements, elements + numElements );
	return elements[selectNo];
}

// main�֐�
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
