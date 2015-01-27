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
    static const unsigned BTS_MTU_SIZE_DEFAULT         = 23;

    /**
    * @param[ref] ble
    *                 BLEDevice object for the underlying controller.
    */
    BlockTransferService(BLEDevice &_ble, block_server_handler_t _readHandler, block_server_handler_t _writeHander);

    ble_error_t updateCharacteristicValue(const uint8_t *value, uint16_t size);

private:
    /* Authorize requests are characteristic specific. No need to check if the handle is correct. */
    void onReadRequest(GattCharacteristicReadAuthCBParams* params);

    /* Authorize requests are characteristic specific. No need to check if the handle is correct. */
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

    void sendReadReply(void);
    bool sendReadReplyRepeatedly(void);

    void requestMissing(void);


private:
    BLEDevice          &ble;
    block_server_handler_t readRequestHandler;
    block_server_handler_t writeDoneHandler;

    uint8_t             receiveBuffer[BTS_MTU_SIZE_DEFAULT];
    uint8_t             sendBuffer[BTS_MTU_SIZE_DEFAULT];
    uint8_t             extraBuffer[BTS_MTU_SIZE_DEFAULT];

    GattCharacteristic*  writeToCharacteristic;
    GattCharacteristic*  readFromCharacteristic; 

    GattAttribute::Handle_t readFromHandle;
    GattAttribute::Handle_t writeToHandle;

    uint16_t receiveBlockOffset;
    uint16_t receiveBlockTotalFragments;

    uint8_t indexBuffer[30];
    index_t receiveBlockMissingFragments;

    block_t* writeBlock;
    block_t writeBlockData;
    uint8_t receiveBlockBuffer[200];

    block_t* readBlock;
    block_t readBlockData;
    
    uint8_t directBlock[BTS_MTU_SIZE_DEFAULT];
    uint16_t readTotalFragments;
    uint16_t readFragmentIndex;
    uint16_t readFragmentsInBatch;    
};

#endif /* #ifndef __BLOCKTRANSFERSERVICE_H__ */
