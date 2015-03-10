#include "ble-blocktransfer/IndexSet.h"


#include "mbed.h"

#define NEED_CONSOLE_OUTPUT 0 /* Set this if you need debug messages on the console;
                               * it will have an impact on code-size and power consumption. */

#if NEED_CONSOLE_OUTPUT
extern Serial  pc;
#define DEBUG(...) { pc.printf(__VA_ARGS__); }
#else
#define DEBUG(...) /* nothing */
#endif /* #if NEED_CONSOLE_OUTPUT */



void initIndexSet(index_t* indexSet, uint8_t* buffer, uint16_t length)
{
    indexSet->buffer = buffer;
    indexSet->bufferLength = length;
}

uint8_t setIndexSetSize(index_t* indexSet, uint16_t size)
{
    uint16_t stop = size / 8;

    if (stop * 8 < size)
    {
        stop++;
    }

    if (stop < indexSet->bufferLength)
    {
        indexSet->indexSize = size;
        indexSet->count = size;

        for (uint16_t idx = 0; idx < stop; idx++)
        {
            indexSet->buffer[idx] = 0xFF;
        }

        return 0; // success
    }
    else
    {
        return 1; // failed
    }
}


void removeIndex(index_t* indexSet, uint16_t index)
{
    if (index < indexSet->indexSize)
    {
        uint16_t byte = index / 8;
        uint8_t bit = index - (byte * 8);

        if ((indexSet->buffer[byte] >> bit) & 0x01)
        {
            indexSet->buffer[byte] &= ~(0x01 << bit);
            indexSet->count--;
        }
    }
}


static uint8_t findSimilar(index_t* indexSet, uint16_t* index, uint8_t value);

void findMissing(index_t* indexSet, uint16_t* index, uint16_t* count)
{
    uint16_t start = 0;
    uint16_t end;
    uint8_t retval;

    retval = findSimilar(indexSet, &start, 1);

    if (retval == 0)
    {
        end = start + 1;

        retval = findSimilar(indexSet, &end, 0);

        if (retval == 0)
        {
            // found both beginning and end
            *index = start;
            *count = end - start;
        }
        else
        {
            // found only the beginning, use length for count
            *index = start;
            *count = indexSet->indexSize - start;
        }
    }
    else
    {
        *count = 0;
    }
}



static uint8_t findSimilar(index_t* indexSet, uint16_t* index, uint8_t value)
{
    uint16_t byte = *index / 8;
    uint8_t bit = *index - (byte * 8);

    uint16_t bufferInUse = indexSet->indexSize / 8;

    if (bufferInUse * 8 < indexSet->indexSize)
    {
        bufferInUse++;
    }

    for ( ; byte < bufferInUse; byte++)
    {
        for ( ; bit < 8; bit++)
        {
            DEBUG("%02X\n\r", (0x01 << bit) & indexSet->buffer[byte]);

            if (((0x01 << bit) & indexSet->buffer[byte]) == (value << bit))
            {
                DEBUG("break %d\n\r", bit);

                uint16_t tmp = byte * 8 + bit;

                if (tmp < indexSet->indexSize)
                {
                    *index = tmp;

                    return 0; // success
                }
                else
                {
                    return 1; // overshot
                }
            }
        }

        bit = 0;
    }

    return 1; // not found
}


bool containsIndex(index_t* indexSet, uint16_t index)
{
    uint16_t byte = index / 8;
    uint8_t bit = index - (byte * 8);

    return (((indexSet->buffer[byte] >> bit) & 0x01) == 1);
}



