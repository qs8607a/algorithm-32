/*
quicksort
- http://ja.wikipedia.org/wiki/%E3%82%AF%E3%82%A4%E3%83%83%E3%82%AF%E3%82%BD%E3%83%BC%E3%83%88
- http://stricter.org/docs/tech/quicksort/
*/

#include <cstdio>
#include <algorithm>

// quick sort���s���B������j�󂷂邱�Ƃɒ���
template<typename T,typename Pred>
static void quickSort( T* elements, size_t numElements, Pred pred )
{
	// �v�f����ȉ��̏ꍇ�̓\�[�g����K�v���Ȃ��̂ŉ������Ȃ�
	if( numElements < 2 )
	{ return ; }
	/*
	pivot�̌�����s���B
	�u�擪�̒l���g���v�u�����ɂ���l���g���v�u�擪�Ɠ�ڂ̒l���ב傫���ق����g���v
	�Ȃǂ̑f�p�ȕ��@����Ƃ��ďオ�邱�Ƃ��������A�S�ē��l�̗񂪗����ꍇ�ɖ����ċA�ɂȂ��Ă��܂��B
	�u�S�ē��l�ɂȂ��Ă��܂��Ă��邩�v�̃`�F�b�N�Ɓu�s�{�b�g�͏��Ȃ��Ƃ��Œ�l�ł͂Ȃ��v�̃`�F�b�N���ǂ����Ă��K�v�B
	*/
	int numSameNumber = 0;
	while(
		!pred( elements[numSameNumber],elements[numSameNumber+1] ) && 
		!pred( elements[numSameNumber+1],elements[numSameNumber] ) &&
		numSameNumber < numElements )
	{
		++numSameNumber;
	}
	// ���l�݂̗̂�Ȃ̂Ń\�[�g�̕K�v�Ȃ�
	if( numSameNumber + 1 == numElements )
	{
		return ;
	}
	// ���v�f�̌���(�ׂ����l���Ⴄ�l�ŁA���̍����ق��ł���Η񒆂̍Œ�l�ł͊m���ɂȂ�)���A�Ō���Ɉړ�
	if( elements[numSameNumber] < elements[numSameNumber+1] )
	{ std::swap( elements[numSameNumber+1], elements[numElements-1] ); }
	else
	{ std::swap( elements[numSameNumber], elements[numElements-1] ); }
	T pivotValue = elements[numElements-1];
	// ��̗v�f�ɕ���
	int lowArrayIndex = 0;
	for(int i=0;i<numElements;++i)
	{
		if( pred( elements[i], pivotValue ) )
		{
			std::swap( elements[lowArrayIndex], elements[i] );
			++lowArrayIndex;
		}
	}
	// ���������f�[�^�������Ƀ\�[�g
	quickSort( elements, lowArrayIndex, pred );
	quickSort( elements + lowArrayIndex, numElements - lowArrayIndex, pred );
}

// main�֐�
void main()
{
	int elements[] = {8,2,9,2,3,9};
	quickSort( elements, sizeof(elements)/sizeof(*elements), [](int x,int y){ return x<y; } );
}
