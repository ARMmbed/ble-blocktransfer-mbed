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

#ifndef __BLOCKTRANSFERSERVICE_H__
#define __BLOCKTRANSFERSERVICE_H__

#include "mbed-drivers/mbed.h"
#include "ble/BLE.h"

#include "ble-blocktransfer/BlockTransfer.h"
#include "ble-blocktransfer/IndexSet.h"
#include "ble-blocktransfer/FunctionPointerWithContext.h"
#include "ble-blocktransfer/FunctionPointerWithContextAndReturnValue.h"
#include "ble-blocktransfer/SharedPointer.h"

#include "mbed-block/BlockDynamic.h"

using namespace mbed::util;

/**
* @class BlockTransferService
* @brief BLE Service for transferring large blocks of data over BLE
*/
class BlockTransferService {

public:
//    static const unsigned BTS_MTU_SIZE_DEFAULT         = 20;

    /* state machine */
    typedef enum {
        BT_STATE_SERVER_READ,
        BT_STATE_SERVER_WRITE,
        BT_STATE_SERVER_WRITE_ACK,
        BT_STATE_READY,
        BT_STATE_OFF
    } bt_state_t;

    BlockTransferService();

    /**
    * @param        &uuid           Service UUID for the Block Transfer Service.
    * @param        securityMode    Security mode required.
    */
    void init(UUID uuid,
              SecurityManager::SecurityMode_t securityMode
              = SecurityManager::SECURITY_MODE_ENCRYPTION_OPEN_LINK);

    /*
    * Set "write received" callback function and write buffer.
    *
    * The function is called when a block of data has been written to the write buffer.
    * The callback can either return the same block or swap it with a different one.
    *
    * @param        writeHandler    This function is called when data has been written to the
    *                               writeCharacteristic. The function can either return the same
    *                               Block pointer or swap it with a different one for better
    *                               memory management.
    */
    void setWriteAuthorizationCallback(void (*writeHandler)(SharedPointer<BlockStatic>));

    template <typename T>
    void setWriteAuthorizationCallback(T* object, void (T::*member)(SharedPointer<BlockStatic>))
    {
        /*  Guard against resetting callback and write block while in the middle of a transfer. */
        if (writeState == BT_STATE_READY)
        {
            writeDoneHandler.attach(object, member);
        }
    }

    /*
    * Set "read requested" callback function.
    *
    * The function is called when a client wants to read the Block Transfer service.
    * The function can modify the Block fields 'uint8_t* data' and 'uint32_t length'
    * to return the data to be sent back to the client.
    *
    * Setting the length to '0' means the read request was denied.
    *
    * @param        readHandler     This function is called when a readCharacteristic is received
    *                               The function updates the readBlock datastructure with the data
    *                               to be read. The function can also refuse the operation by
    *                               setting the length to zero.
    */
    void setReadAuthorizationCallback(SharedPointer<Block> (*readHandler)(uint32_t));

    template <typename T>
    void setReadAuthorizationCallback(T* object, SharedPointer<Block> (T::*member)(uint32_t))
    {
        /*  Guard against resetting callback while in the middle of a transfer. */
        if (readState == BT_STATE_READY)
        {
            readRequestHandler.attach(object, member);
        }
    }

    /* Send a short direct message to the client. Replacement for Handle Value Indications. */
    ble_error_t updateCharacteristicValue(const uint8_t* value, uint16_t size);

    /* Register callback for when a read request has finished */
    void setReadDoneCallback(void (*readDoneHandler)(void));

    template <typename T>
    void setReadDoneCallback(T* object, void (T::*member)(void))
    {
        if ((readState == BT_STATE_OFF) || (readState == BT_STATE_READY))
        {
            readDoneHandler.attach(object, member);
        }
    }

    /* Check if service is ready */
    bool writeInProgess(void);
    bool readInProgress(void);
    bool isReady(void);

private:
    /* Initialization */
    void initDone(BLE::InitializationCompleteCallbackContext* context);

    /* Internal callback functions for handling read and write requests. */
    void onReadRequest(GattReadAuthCallbackParams* params);
    void onWriteRequest(GattWriteAuthCallbackParams* params);

    /*  This function is called when the BLE device is ready for more characteristic value updates
        and is shared by all characteristics.
    */
    void onDataSent(unsigned count);

    /*  This function is called when any writes have been received by the BLE device.
        Must filter on characteristic handle to ensure we only respond to writes
        intended for this characteristic.
    */
    void onDataWritten(const GattWriteCallbackParams* event);

    /* Connection status callbacks. Sets and resets variables and state. */
    void onConnection(const Gap::ConnectionCallbackParams_t* params);
    void onDisconnection(const Gap::DisconnectionCallbackParams_t* params);

    /*  Functions for feeding individual fragments in each batch.
    */
    void sendReadReply(void);
    bool sendReadReplyRepeatedly(void);

    void sendWriteRequestMissing(void);
    void sendWriteAcknowledgement(void);

    /*  Timeout function called when fragments are not received in time.
    */
    void fragmentTimeout(void);

private:
    BLE ble;
    UUID uuid;
    SecurityManager::SecurityMode_t securityMode;

    /*  Connection handle for transfer in progress.
    */
    Gap::Handle_t connectionHandle;
    uint8_t connectionCounter;

    /*  Handles for callback functions.
    */
    FunctionPointerWithContextAndReturnValue<uint32_t, SharedPointer<Block> > readRequestHandler;
    FunctionPointerWithContext<SharedPointer<BlockStatic> >                   writeDoneHandler;
    FunctionPointer                                                           readDoneHandler;

    /*  Pointer to a data structure which contains the writeTo (receive) buffer.
    */
    SharedPointer<BlockStatic> writeBlock;

    /*  BLE characteristics the block transfer is built upon.
    */
    GattCharacteristic readFromCharacteristic;
    GattCharacteristic writeToCharacteristic;

    GattAttribute::Handle_t readFromHandle;
    GattAttribute::Handle_t writeToHandle;

    /*  Data structure pointing to the read characteristic value.
        This data structure is updated everytime a read request is received.
    */
    SharedPointer<Block> readBlock;

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
    IndexSet<MAX_INDEX_SET_SIZE> missingFragments;

#if defined(YOTTA_MINAR_VERSION_STRING)
    minar::callback_handle_t timeoutHandle;
#endif

    /*  Internal variable containing the current MTU size.
    */
    uint16_t currentMTU;
    uint16_t maxBlockPayloadSize;
    uint16_t maxDirectReadPayloadSize;

    /*
    */
    bt_state_t readState;
    bt_state_t writeState;
};

#endif /* #ifndef __BLOCKTRANSFERSERVICE_H__ */
