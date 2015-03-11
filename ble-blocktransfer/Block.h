#ifndef __BLOCK_H__
#define __BLOCK_H__


typedef struct
{
    uint8_t* data;
    uint16_t length;
    uint16_t offset;
    uint16_t maxLength;
} block_t;

#endif // __BLOCK_H__
