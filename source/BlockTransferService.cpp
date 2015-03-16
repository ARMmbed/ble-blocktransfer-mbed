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

//#include "BlockTransfer.h"
#include "ble-blocktransfer/BlockTransferService.h"

#if 0
#define BLE_DEBUG(...) { printf(__VA_ARGS__); }
#else
#define BLE_DEBUG(...) /* nothing */
#endif

const uint16_t EnvoyServiceWriteCharacteristicShortUUID = 0x0001;
const uint16_t EnvoyServiceReadCharacteristicShortUUID  = 0x0002;


BlockTransferService::BlockTransferService(BLEDevice &_ble,
                                           block_read_handler_t _readHandler,
                                           block_write_handler_t _writeHander,
                                           block_t* _writeBlock)
    :   ble(_ble),
        readRequestHandler(_readHandler),
        writeDoneHandler(_writeHander),
        writeBlock(_writeBlock),
        receiveBuffer(),
        sendBuffer(),
        receiveBlockOffset(0),
        receiveBlockTotalFragments(0)
{
    /*  Setup standard BLE read and write characteristics on which the block transfer protocol is built upon.
    */
    readFromCharacteristic = new GattCharacteristic(EnvoyServiceReadCharacteristicShortUUID,
                                        sendBuffer, 1, BTS_MTU_SIZE_DEFAULT,
                                        GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ |
                                        GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY);
    writeToCharacteristic = new GattCharacteristic(EnvoyServiceWriteCharacteristicShortUUID,
                                        receiveBuffer, 1, BTS_MTU_SIZE_DEFAULT,
                                        GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE |
                                        GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE_WITHOUT_RESPONSE);

    readFromCharacteristic->setReadAuthorizationCallback(this, &BlockTransferService::onReadRequest);
    writeToCharacteristic->setWriteAuthorizationCallback(this, &BlockTransferService::onWriteRequest);

    GattCharacteristic *charTable[] = {readFromCharacteristic, writeToCharacteristic};
    GattService BTService(UUID(0xFFFF), charTable, sizeof(charTable) / sizeof(GattCharacteristic *));

    ble.addService(BTService);

    // handles are set when the service has been added to the ble device
    readFromHandle = readFromCharacteristic->getValueHandle();
    writeToHandle = writeToCharacteristic->getValueHandle();

    // register callback functions
    ble.onDataWritten(this, &BlockTransferService::onDataWritten);
    ble.onDataSent(this, &BlockTransferService::onDataSent);
    ble.addToDisconnectionCallChain(this, &BlockTransferService::onDisconnection);

    // datastructure for setting up the read (send) buffer
    readBlock = &readBlockData;

    // initialize bitmap for keeping track of fragments
    initIndexSet(&receiveBlockMissingFragments, indexBuffer, 30);
}

ble_error_t BlockTransferService::updateCharacteristicValue(const uint8_t *value, uint16_t size)
{
    ble_error_t returnValue = BLE_ERROR_BUFFER_OVERFLOW;

    if (size <= MAX_DIRECT_READ_PAYLOAD_SIZE)
    {
        // insert header
        uint8_t payload[DIRECT_READ_HEADER_SIZE + size];
        payload[0] = BT_TYPE_READ_NOTIFY;

        memcpy(&payload[1], value, size);

        returnValue = ble.updateCharacteristicValue(readFromHandle, payload, DIRECT_READ_HEADER_SIZE + size);
    }

    return returnValue;
}


/* Authorize requests are characteristic specific. No need to check if the handle is correct. */
void BlockTransferService::onReadRequest(GattCharacteristicReadAuthCBParams* event)
{
/*
    GattAttribute::Handle_t  charHandle;
    uint16_t                 offset;
    bool                     authorizationReply;
*/
    BLE_DEBUG("read request\n\r");

    if (event->charHandle == readFromHandle)
    {
        readBlock->offset = event->offset;

        /* call higher layer for read value */
        readRequestHandler(readBlock);

        // read denied
        if (readBlock->length == 0)
        {
            event->authorizationReply = false;
        }
        // send the message directly if it can fit in one packet
        else if (readBlock->length < 20)
        {
            directBlock[0] = BT_TYPE_READ_DIRECT;

            memcpy(&directBlock[1], readBlock->data, readBlock->length);

            event->authorizationReply = true;
            event->data = directBlock;
            event->len = readBlock->length + 1;
        }
        else
        {
            // guard against retransmission received in the wrong state
            if (blockTransferState == BT_STATE_OFF)
            {
                // the block is too large to fit in one packet.
                // Respond with a setup packet specifying the number of fragments available

                readTotalFragments = readBlock->length / MAX_BLOCK_PAYLOAD_SIZE;
                if (readTotalFragments * MAX_BLOCK_PAYLOAD_SIZE < readBlock->length)
                {
                    readTotalFragments++;
                }

                blockTransferState = BT_STATE_SERVER_READ;
            }

            /*  Respond with setup message.
                When the receiver is ready for data it will request fragments.
            */
            directBlock[0] = BT_TYPE_READ_SETUP;
            directBlock[1] = readBlock->length; // LSB
            directBlock[2] = readBlock->length >> 8;
            directBlock[3] = readTotalFragments; // LSB
            directBlock[4] = readTotalFragments >> 8;

            event->authorizationReply = true;
            event->data = directBlock;
            event->len = 5;
        }
    }
}


/*  This function is called when any writes have been received by the BLE device.
    Must filter on characteristic handle to ensure we only respond to writes
    intended for this characteristic.
*/
void BlockTransferService::onDataWritten(const GattCharacteristicWriteCBParams* event)
{
/*
    event->charHandle;
    event->op;
    event->offset;
    event->len;
    event->data;
*/
    if (event->charHandle == writeToHandle)
    {
        const uint8_t* message = event->data;
        const bt_type_t messageType = (bt_type_t) message[0];

        // TODO store time stamp for timeouts
        // lastReceivedMessage = [NSDate timeIntervalSinceReferenceDate];
        switch(messageType)
        {
            case BT_TYPE_WRITE_SETUP:
                {
                    // guard against retransmissions resetting the state
                    if (blockTransferState == BT_STATE_OFF)
                    {
                        /*  Setup message received.
                        */
                        // reset receive buffer so full length
                        writeBlock->length = writeBlock->maxLength;
                        writeBlock->offset = 0;

                        // block length, number is LSB
                        uint16_t blockLength;
                        blockLength = message[2];
                        blockLength = (blockLength << 8) | message[1];

                        if (blockLength <= writeBlock->length)
                        {
                            // the offset of the current block with regards to the overall characteristic
                            receiveBlockOffset = message[4];
                            receiveBlockOffset = (receiveBlockOffset << 8) | message[3];

                            // fragments in block, number is send LSB
                            receiveBlockTotalFragments = message[6];
                            receiveBlockTotalFragments = (receiveBlockTotalFragments << 8) | message[5];

                            BLE_DEBUG("MTU %d %d %d\r\n", blockLength, receiveBlockTotalFragments, MAX_BLOCK_PAYLOAD_SIZE);

                            writeBlock->length = blockLength;
                            writeBlock->offset = receiveBlockOffset;

                            // use IndexSet to keep track of the received fragments and find those missing
                            setIndexSetSize(&receiveBlockMissingFragments, receiveBlockTotalFragments);

                            blockTransferState = BT_STATE_SERVER_WRITE;
                        }
                        else
                        {
                            // use IndexSet to keep track of the received fragments and find those missing
                            setIndexSetSize(&receiveBlockMissingFragments, 0);
                        }
                    }

                    // acknowledge setup by requesting data
                    requestMissing();

                    BLE_DEBUG("setup fragments (%d) (%d)\r\n", receiveBlockTotalFragments, event->len);
                }

                break;

            case BT_TYPE_WRITE_DIRECT:
                {
                    /*  Direct message received.
                        Send payload to upper layer.
                    */
                    // the offset of the current block with regards to the overall characteristic
                    receiveBlockOffset = message[2];
                    receiveBlockOffset = (receiveBlockOffset << 8) | message[1];

                    // discard header, grab payload
                    memcpy(writeBlock->data, &event->data[3], event->len - 3);
                    writeBlock->offset = receiveBlockOffset;
                    writeBlock->length = event->len - 3;

                    // Full block received. No change in state.
                    // signal upper layer of write request
                    writeBlock = writeDoneHandler(writeBlock);
                }
                break;

            case BT_TYPE_WRITE_PAYLOAD_MORE:
            case BT_TYPE_WRITE_PAYLOAD_LAST:
                {
                    if (blockTransferState == BT_STATE_SERVER_WRITE)
                    {
                        uint16_t fragmentNumber;

                        // number is LSB
                        fragmentNumber = message[2];
                        fragmentNumber = (fragmentNumber << 8) | message[1];

                        if (containsIndex(&receiveBlockMissingFragments, fragmentNumber))
                        {
                            BLE_DEBUG("%d\r\n", fragmentNumber);

                            // mark fragment as received in the index
                            removeIndex(&receiveBlockMissingFragments, fragmentNumber);

                            // copy payload to receive buffer
                            uint16_t index = fragmentNumber * MAX_BLOCK_PAYLOAD_SIZE;
                            memcpy(&writeBlock->data[index], &event->data[3], event->len - 3);

                            // when sender signals "no more data", request missing fragments
                            // or stop the timer and signal reception complete
                            // if the last message in a batch is lost the timeout will
                            // re-request missing fragments
                            if (messageType == BT_TYPE_WRITE_PAYLOAD_LAST)
                            {
                                if (receiveBlockMissingFragments.count == 0)
                                {
                                    // signal upper layer of write request
                                    writeBlock = writeDoneHandler(writeBlock);

                                    blockTransferState = BT_STATE_OFF;
                                }

                                // request missing fragments or acknowledge the reception of the last fragment
                                requestMissing();
                            }
                        }
                        else
                        {
                            BLE_DEBUG("duplicate %d\r\n", fragmentNumber);
                        }
                    }
                }
                break;

            case BT_TYPE_READ_REQUEST:
                {
                    uint16_t sendFromFragment;
                    uint16_t sendAmount;

                    // fragments in block, number is send LSB
                    sendFromFragment = message[2];
                    sendFromFragment = (sendFromFragment << 8) | message[1];

                    // the offset of the current block with regards to the overall characteristic
                    sendAmount = message[4];
                    sendAmount = (sendAmount << 8) | message[3];

                    if (sendFromFragment < readTotalFragments)
                    {
                        readFragmentIndex = sendFromFragment;
                        readFragmentsInBatch = sendAmount;

                        BLE_DEBUG("read request %d %d %d\r\n", sendFromFragment, sendAmount, readTotalFragments);

                        sendReadReply();
                    }
                    else
                    {
                        blockTransferState = BT_STATE_OFF;
                        BLE_DEBUG("read complete\r\n");
                    }
                }
                break;

            default:
                BLE_DEBUG("error - unknown message type (%02X)\r\n", messageType);
                break;
        }
    }
    else
    {
        BLE_DEBUG("written: %04X %04X\r\n", event->charHandle, writeToHandle);
    }
}

/* Connection disconnected. Reset variables and state. */
void BlockTransferService::onDisconnection()
{
    BLE_DEBUG("BLE: disconnected\r\n");

    blockTransferState = BT_STATE_OFF;
}

/*  This function is called when the BLE device is ready for more characteristic value updates
    and is shared by all characteristics.
*/
void BlockTransferService::onDataSent(unsigned count)
{
    BLE_DEBUG("sent: %d\r\n", count);

    if (blockTransferState == BT_STATE_SERVER_READ)
    {
        sendReadReply();
    }
}


void BlockTransferService::sendReadReply(void)
{
    BLE_DEBUG("send read reply\r\n");

    while (readFragmentsInBatch != 0)
    {
        bool result = sendReadReplyRepeatedly();

        BLE_DEBUG("notify: %d %d\r\n", readFragmentIndex, result);

        if (result)
        {
            readFragmentIndex++;
            readFragmentsInBatch--;
        }
        else
        {
            break;
        }
    }
}


bool BlockTransferService::sendReadReplyRepeatedly(void)
{
    // find length. the last packet might be shorter.
    // readFragmentIndex is zero-indexed
    uint16_t length = (readFragmentIndex < readTotalFragments - 1) ? MAX_BLOCK_PAYLOAD_SIZE :
                readBlock->length - ((readTotalFragments - 1) * MAX_BLOCK_PAYLOAD_SIZE);

    // set data type based on whether more data is pending in this batch
    uint8_t type = (readFragmentsInBatch == 1) ? BT_TYPE_READ_PAYLOAD_LAST : BT_TYPE_READ_PAYLOAD_MORE;

    // insert header
    uint8_t payload[BLOCK_HEADER_SIZE + length];
    payload[0] = type;
    payload[1] = readFragmentIndex;
    payload[2] = readFragmentIndex >> 8;

    uint16_t index = readFragmentIndex * MAX_BLOCK_PAYLOAD_SIZE;
    memcpy(&payload[3], &readBlock->data[index], length);

    ble_error_t didSendValue = ble.updateCharacteristicValue(readFromHandle, payload, BLOCK_HEADER_SIZE + length);

    return (didSendValue == BLE_ERROR_NONE);
}


void BlockTransferService::requestMissing(void)
{
    BLE_DEBUG("request missing\r\n");

    if (receiveBlockMissingFragments.count > 0)
    {
        // find missing ranges
        uint16_t fragmentNumber;
        uint16_t count;

        findMissing(&receiveBlockMissingFragments, &fragmentNumber, &count);

        uint8_t request[5];

        request[0] = BT_TYPE_WRITE_REQUEST;
        request[1] = fragmentNumber;
        request[2] = fragmentNumber >> 8;
        request[3] = count;
        request[4] = count >> 8;

        ble.updateCharacteristicValue(readFromHandle, request, 5);

        BLE_DEBUG("send ack: %d %d\r\n", fragmentNumber, count);
    }
    else
    {
        // send acknowledgment for last fragment
        uint8_t request[5];

        request[0] = BT_TYPE_WRITE_REQUEST;
        request[1] = 0xFF;
        request[2] = 0xFF;
        request[3] = 0x01;
        request[4] = 0x00;

        ble.updateCharacteristicValue(readFromHandle, request, 5);

        BLE_DEBUG("send ack: 0xFFFF 1\r\n");
    }
}




/* Authorize requests are characteristic specific. No need to check if the handle is correct. */
void BlockTransferService::onWriteRequest(GattCharacteristicWriteAuthCBParams* params)
{
/*
    GattAttribute::Handle_t  charHandle;
    uint16_t                 offset;
    uint16_t                 len;
    const uint8_t           *data;
    bool                     authorizationReply;
*/
    BLE_DEBUG("write request\n\r");

    if (params->data[0] == BT_TYPE_WRITE_DIRECT)
    {
        /*  Direct message received.
            Send payload to upper layer.
        */

        // the offset of the current block with regards to the overall characteristic
        receiveBlockOffset = params->data[2];
        receiveBlockOffset = (receiveBlockOffset << 8) | params->data[1];

        // discard header, grab payload
        memcpy(writeBlock->data, &params->data[3], params->len - 3);
        writeBlock->offset = receiveBlockOffset;
        writeBlock->length = params->len - 3;

        // Full block received. No change in state.
        // signal upper layer of write request
        writeBlock = writeDoneHandler(writeBlock);

        params->authorizationReply = true;
    }
    else
    {
        params->authorizationReply = false;
    }

}
