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
#include "ble-blocktransfer/Block.h"
#include "ble-blocktransfer/IndexSet.h"

typedef enum
{
    BT_SUCCESS = 0,
    BT_ERROR = 1
} bt_error_t;

void bridgeCharacteristicDiscoveryCallback(const DiscoveredCharacteristic* characteristicP);

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

    /**
    * @param        &ble            BLEDevice object for the underlying controller.
    * @param        &uuid           Service UUID for the Block Transfer Service.
    * @param        securityMode    Security mode required.
    */
    BlockTransferClient(BLE& _ble,
                        const UUID& uuid,
                        Gap::Handle_t,
                        void (*clientReady)(void));

    template <typename T>
    BlockTransferClient(BLE& _ble,
                        const UUID& uuid,
                        Gap::Handle_t _peripheral,
                        T* object,
                        void (T::*member)(void))
        :   ble(_ble),
            peripheral(_peripheral),
            internalState(BT_STATE_OFF)
    {
        writeDoneHandler.attach(object, member);

        ble.gattServer().onDataSent(this, &BlockTransferClient::internalDataSent);

        ble.gattClient().launchServiceDiscovery(peripheral,
                                                NULL,
                                                bridgeCharacteristicDiscoveryCallback,
                                                uuid);
    }


    /*  Read
    */
    bt_error_t read(Block* buffer, void (*readDone)(Block*))
    {
        readDoneHandler.attach(readDone);

        return internalRead(buffer);
    }

    template <typename T>
    bt_error_t read(Block* buffer, T* object, void (T::*member)(Block*))
    {
        readDoneHandler.attach(object, member);

        return internalRead(buffer);
    }

    /*  Write
    */
    bt_error_t write(Block* buffer, void (*writeDone)(void))
    {
        writeDoneHandler.attach(writeDone);

        return internalWrite(buffer);
    }

    template <typename T>
    bt_error_t write(Block* buffer, T* object, void (T::*member)(void))
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

    /*  Helper methods
    */
    void characteristicDiscoveryCallback(const DiscoveredCharacteristic* characteristic);
    void hvxCallback(const GattHVXCallbackParams* params);
    void readCallback(const GattReadCallbackParams* params);

private:
    bt_error_t internalRead(Block* buffer);
    bt_error_t internalWrite(Block* buffer);
    void internalDataSent(unsigned count);

    void internalSendWriteReply();
    bool internalSendWriteReplyRepeatedly();
    void internalSendReadRequest();

    void internalOnDisconnection();
private:
    BLE& ble;
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
    uint16_t incomingPayloadSize;

    /*  Bitmap for keeping track of "missing" fragments.
        Note: if the BLE stack is working properly, fragments should never be missing.
    */
    IndexSet<MAX_INDEX_SET_SIZE> missingFragments;

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
