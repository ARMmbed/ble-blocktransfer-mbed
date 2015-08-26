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

#ifndef __BLOCKTRANSFERCLIENT_H__
#define __BLOCKTRANSFERCLIENT_H__

#include "mbed.h"
#include "ble/BLE.h"
#include "ble/DiscoveredCharacteristic.h"
#include "ble/DiscoveredService.h"

#include "ble-blocktransfer/BlockTransfer.h"
#include "ble-blocktransfer/IndexSet.h"

#include "mbed-block/Block.h"
#include "mbed-block/BlockStatic.h"
#include "mbed-block/BlockStaticReadOnly.h"
#include "mbed-block/BlockCollection.h"

class BlockTransferClient;

void bridgeCharacteristicDiscoveryCallback(const DiscoveredCharacteristic* characteristicP);
void bridgeReadCallback(const GattReadCallbackParams* params);
void bridgeHVXCallback(const GattHVXCallbackParams* params);
extern BlockTransferClient* btcBridge;

class BlockTransferClient
{
public:
    /* state machine */
    typedef enum {
        BT_STATE_CLIENT_SUBSCRIBE,
        BT_STATE_CLIENT_WRITE_DIRECT,
        BT_STATE_CLIENT_WRITE_SETUP,
        BT_STATE_CLIENT_WRITE_PAYLOAD,
        BT_STATE_CLIENT_READ_SETUP,
        BT_STATE_CLIENT_READ_REQUEST,
        BT_STATE_CLIENT_READ_ACK,
        BT_STATE_READY,
        BT_STATE_OFF
    } bt_state_t;

    BlockTransferClient();

    /**
    * @param        &uuid           Service UUID for the Block Transfer Service.
    * @param        securityMode    Security mode required.
    */
    void init(void (*clientReady)(void),
              const UUID& uuid,
              Gap::Handle_t);

    template <typename T>
    void init(T* object,
              void (T::*member)(void),
              const UUID& uuid,
              Gap::Handle_t _peripheral)
    {
        peripheral = _peripheral;

        writeDoneHandler.attach(object, member);

        ble.init();

        ble.gattServer().onDataSent(this, &BlockTransferClient::internalDataSent);

        ble.gattClient().launchServiceDiscovery(peripheral,
                                                NULL,
                                                bridgeCharacteristicDiscoveryCallback,
                                                uuid);

        ble.gattClient().onHVX(bridgeHVXCallback);
        ble.gattClient().onDataRead(bridgeReadCallback);

        ble.gap().addToDisconnectionCallChain(this, &BlockTransferClient::internalOnDisconnection);

        btcBridge = this;
    }


    /*  Read
    */
    ble_error_t read(Block* buffer, void (*readDone)(Block*))
    {
        readDoneHandler.attach(readDone);

        return internalRead(buffer);
    }

    template <typename T>
    ble_error_t read(Block* buffer, T* object, void (T::*member)(Block*))
    {
        readDoneHandler.attach(object, member);

        return internalRead(buffer);
    }

    /*  Write
    */
    ble_error_t write(Block* buffer, void (*writeDone)(void))
    {
        writeDoneHandler.attach(writeDone);

        return internalWrite(buffer);
    }

    template <typename T>
    ble_error_t write(Block* buffer, T* object, void (T::*member)(void))
    {
        writeDoneHandler.attach(object, member);

        return internalWrite(buffer);
    }

    /*  Notications
    */
    void onNotification(void (*_notificationHandler)(void))
    {
        notificationHandler.attach(_notificationHandler);
    }

    template <typename T>
    void onNotification(T* object, void (T::*member)(void))
    {
        notificationHandler.attach(object, member);
    }

    /* Check if client is ready */
    bool writeInProgess(void);
    bool readInProgress(void);
    bool ready(void);

    /*  Helper methods
    */
    void characteristicDiscoveryCallback(const DiscoveredCharacteristic* characteristic);
    void hvxCallback(const GattHVXCallbackParams* params);
    void readCallback(const GattReadCallbackParams* params);

private:
    /* Internal functions for handling read and write requests. */
    ble_error_t internalRead(Block* buffer);
    ble_error_t internalWrite(Block* buffer);

    /*  This function is called when the BLE device is ready to send more messages
        and is shared by all characteristics.
    */
    void internalDataSent(unsigned count);

    /* Connection disconnected. Reset variables and state. */
    void internalOnDisconnection(void);

    /*  Functions for feeding individual fragments in each batch.
    */
    void internalSendWriteReply(void);
    bool internalSendWriteReplyRepeatedly(void);

    void internalSendReadRequest(void);
    void internalSendReadAcknowledgement(void);

    /*  Timeout function called when fragments are not received in time.
    */
    void fragmentTimeout(void);

private:
    BLE ble;
    Gap::Handle_t peripheral;

    FunctionPointerWithContext<Block*> readDoneHandler;
    FunctionPointer writeDoneHandler;
    FunctionPointer notificationHandler;

    DiscoveredCharacteristic readCharacteristic;
    DiscoveredCharacteristic writeCharacteristic;

    Block* writeBlock;
    Block* readBlock;


    /*  Internal variables for keeping track of outgoing fragments in a batch.
    */
    uint8_t directBlock[BTS_MTU_SIZE_DEFAULT];
    uint32_t outgoingTotalFragments;
    uint32_t outgoingFragmentIndex;
    uint32_t outgoingFragmentsInBatch;

    /*  Internal variables for keeping track of incoming fragments in a batch.
    */
    uint32_t incomingTotalFragments;
    uint32_t incomingTotalLength;

    /*  Bitmap for keeping track of "missing" fragments.
        Note: if the BLE stack is working properly, fragments should never be missing.
    */
    IndexSet<MAX_INDEX_SET_SIZE> missingFragments;
    Timeout timeout;

    /*  Internal variable containing the current MTU size.
    */
    uint16_t currentMTU;
    uint16_t maxBlockPayloadSize;
    uint16_t maxDirectReadPayloadSize;

    /*
    */
    bt_state_t internalState;
};

#endif // __BLOCKTRANSFERCLIENT_H__
