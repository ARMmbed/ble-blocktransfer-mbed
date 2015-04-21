#ifndef __BLOCK_H__
#define __BLOCK_H__


typedef struct
{
    uint8_t* data;
    uint32_t length;
    uint32_t offset;
    uint32_t maxLength;
} block_t;

#endif // __BLOCK_H__
