/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
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

#ifndef __BLOCKTRANSFERSERVICE_H__
#define __BLOCKTRANSFERSERVICE_H__

#include "mbed.h"


#include "BlockTransfer.h"
#include "IndexSet.h"

#include "UUID.h"
#include "BLEDevice.h"



/**
* @class BlockTransferService
* @brief BLE Service for transferring large blocks of data over BLE
*/


class BlockTransferService {

public:
    static const unsigned BTS_MTU_SIZE_DEFAULT         = 20;

    /**
    * @param        &ble            BLEDevice object for the underlying controller.
    * @param        &uuid           Service UUID for the Block Transfer Service.
    * @param        readHandler     This function is called when a readCharacteristic is received
    *                               The function updates the readBlock datastructure with the data
    *                               to be read. The function can also refuse the operation by
    *                               setting the length to zero.
    * @param        writeHandler    This function is called when data has been written to the
    *                               writeCharacteristic. The function can either return the same
    *                               block_t pointer or swap it with a different one for better
    *                               memory management.
    * @param        writeBlock      The initial datastructure pointing to the buffer set a side for
    *                               receiving data.
    */
    BlockTransferService(BLEDevice &_ble,
                         const UUID &uuid,
                         block_read_handler_t readHandler,
                         block_write_handler_t writeHander,
                         block_t* writeBlock);

    /* Send a short direct message to the client. Replacement for Handle Value Indications. */
    ble_error_t updateCharacteristicValue(const uint8_t *value, uint16_t size);

private:
    /* Internal callback functions for handling read and write requests. */
    void onReadRequest(GattCharacteristicReadAuthCBParams* params);
    void onWriteRequest(GattCharacteristicWriteAuthCBParams* params);

    /*  This function is called when the BLE device is ready for more characteristic value updates
        and is shared by all characteristics.
    */
    void onDataSent(unsigned count);

    /*  This function is called when any writes have been received by the BLE device.
        Must filter on characteristic handle to ensure we only respond to writes
        intended for this characteristic.
    */
    void onDataWritten(const GattCharacteristicWriteCBParams* event);

    /* Connection disconnected. Reset variables and state. */
    void onDisconnection();

    /*  Functions for feeding individual fragments in each batch.
    */
    void sendReadReply(void);
    bool sendReadReplyRepeatedly(void);
    void requestMissing(void);

private:
    BLEDevice &ble;

    /*  Handles for callback functions.
    */
    block_read_handler_t readRequestHandler;
    block_write_handler_t writeDoneHandler;

    /*  Pointer to a data structure which contains the writeTo (receive) buffer.
        Upon reception, the pointer can be used for swapping buffers instead of copying them.
    */
    block_t* writeBlock;

    /*  BLE characteristics the block transfer is built upon.
    */
    uint8_t receiveBuffer[BTS_MTU_SIZE_DEFAULT];
    uint8_t sendBuffer[BTS_MTU_SIZE_DEFAULT];

    GattCharacteristic*  writeToCharacteristic;
    GattCharacteristic*  readFromCharacteristic;

    GattAttribute::Handle_t readFromHandle;
    GattAttribute::Handle_t writeToHandle;

    /*  Data structure pointing to the read characteristic value.
        This data structure can be updated everytime a read request is received.
    */
    block_t* readBlock;
    block_t readBlockData;

    /*  Internal variables for keeping track of how many fragments have been read in a batch.
    */
    uint8_t directBlock[BTS_MTU_SIZE_DEFAULT];
    uint32_t readTotalFragments;
    uint32_t readFragmentIndex;
    uint32_t readFragmentsInBatch;

    /*  Internal variables for keeping track of how many fragments have been written in a batch.
    */
    uint32_t receiveLengthOffset;
    uint32_t receiveFragmentOffset;
    uint32_t receiveTotalFragments;

    /*  Bitmap for keeping track of "missing" fragments.
        Note: if the BLE stack is working properly, fragments should never be missing.
    */
    uint8_t indexBuffer[30];
    index_t receiveBlockMissingFragments;

    /*  Internal variable containing the current MTU size.
    */
    uint16_t currentMTU;
    uint16_t maxBlockPayloadSize;
    uint16_t maxDirectReadPayloadSize;

    /*
    */
    bt_state_t internalState;
};

#endif /* #ifndef __BLOCKTRANSFERSERVICE_H__ */
