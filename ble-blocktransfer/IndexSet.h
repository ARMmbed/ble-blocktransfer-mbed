/* mbed Microcontroller Library
 * Copyright (c) 2006-2015 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __INDEXSET_H__
#define __INDEXSET_H__

template <std::size_t LENGTH>
class IndexSet
{
public:
    void setSize(uint32_t bitmapInBits)
    {
        // set count to original size regardless of cropping
        count = bitmapInBits;

        // convert bitmap to bytes
        uint32_t bitmapInBytes = bitmapInBits / 8;

        if (bitmapInBytes * 8 < bitmapInBits)
        {
            bitmapInBytes++;
        }

        // crop bitmap to fit index buffer
        if (bitmapInBytes > LENGTH)
        {
            bitmapInBytes = LENGTH;
            bitmapInBits = bitmapInBytes * 8;
        }

        // set actual size
        indexSize = bitmapInBits;

        // set bits in bitmap
        for (uint32_t idx = 0; idx < bitmapInBytes; idx++)
        {
            buffer[idx] = 0xFF;
        }
    }

    void removeIndex(uint32_t index)
    {
        if (index < indexSize)
        {
            // find index location in bitmap
            uint16_t byte = index / 8;
            uint8_t bit = index - (byte * 8);

            // if bit is set, remove and decement counter
            if ((buffer[byte] >> bit) & 0x01)
            {
                buffer[byte] &= ~(0x01 << bit);
                count--;
            }
        }
        else
        {
            // index is out of bounce, decrement count only
            if (count > 0)
            {
                count--;
            }
        }
    }

    void findMissing(uint32_t* index, uint32_t* items)
    {
        uint32_t start = 0;

        // find first-index-still-set
        bool retval = findFirst(&start, 1);

        // found first-index-still-set
        if (retval)
        {
            uint32_t end = start + 1;

            // find last-index-not-set
            retval = findFirst(&end, 0);

            // found last-index-not-set
            if (retval)
            {
                // found both beginning and end
                *index = start;
                *items = end - start;
            }
            else
            {
                // found only the beginning, use remaining count
                *index = start;
                *items = count;
            }
        }
        else
        {
            // all bits have been unset
            *items = 0;
        }
    }

    bool findFirst(uint32_t* index, uint8_t value)
    {
        // index in buffer
        uint32_t byte = *index / 8;
        uint8_t bit = *index - (byte * 8);

        // byte in buffer containing last index
        uint32_t bufferInUse = indexSize / 8;

        if (bufferInUse * 8 < indexSize)
        {
            bufferInUse++;
        }

        // iterate through buffer, one byte at a time
        for ( ; byte < bufferInUse; byte++)
        {
            // iterate through byte, one bit at a time
            for ( ; bit < 8; bit++)
            {
                // compare bit with value
                if (((0x01 << bit) & buffer[byte]) == (value << bit))
                {
                    uint16_t tmp = byte * 8 + bit;

                    // check if we are within bounds before returning
                    if (tmp < indexSize)
                    {
                        *index = tmp;

                        return true; // success
                    }
                    else
                    {
                        return false; // overshot
                    }
                }
            }

            bit = 0;
        }

        return false; // not found
    }


    bool containsIndex(uint32_t index)
    {
        if (index < indexSize)
        {
            uint32_t byte = index / 8;
            uint8_t bit = index - (byte * 8);

            return (((buffer[byte] >> bit) & 0x01) == 1);
        }
        else
        {
            return false;
        }
    }

    uint32_t getCount()
    {
        return count;
    }

private:
    uint8_t  buffer[LENGTH];
    uint32_t indexSize;
    uint32_t count;
};

#endif
