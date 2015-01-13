#include <stdint.h>


#ifdef __cplusplus 
//extern "C" {
#endif

typedef struct
{
  uint8_t* buffer;
  uint16_t bufferLength;
  uint16_t indexSize;
  uint16_t count;
} index_t;

void initIndexSet(index_t* indexSet, uint8_t* buffer, uint16_t length);
uint8_t setIndexSetSize(index_t* indexSet, uint16_t size);
void removeIndex(index_t* indexSet, uint16_t index);

void findMissing(index_t* indexSet, uint16_t* index, uint16_t* count);

bool containsIndex(index_t* indexSet, uint16_t index);

#ifdef __cplusplus 
//}
#endif