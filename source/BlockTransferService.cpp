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

#include "ble-blocktransfer/BlockTransferService.h"

#if 0
#define BLE_DEBUG(...) { printf(__VA_ARGS__); }
#else
#define BLE_DEBUG(...) /* nothing */
#endif

const uint16_t ServiceWriteCharacteristicShortUUID = 0x0001;
const uint16_t ServiceReadCharacteristicShortUUID  = 0x0002;

BlockTransferService::BlockTransferService(BLEDevice &_ble,
                                           const UUID &uuid,
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
    readFromCharacteristic = new GattCharacteristic(ServiceReadCharacteristicShortUUID,
                                        sendBuffer, 1, BTS_MTU_SIZE_DEFAULT,
                                        GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ |
                                        GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY);
    writeToCharacteristic = new GattCharacteristic(ServiceWriteCharacteristicShortUUID,
                                        receiveBuffer, 1, BTS_MTU_SIZE_DEFAULT,
                                        GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE_WITHOUT_RESPONSE);

    readFromCharacteristic->setReadAuthorizationCallback(this, &BlockTransferService::onReadRequest);

    GattCharacteristic *charTable[] = {readFromCharacteristic, writeToCharacteristic};
    GattService BTService(uuid, charTable, sizeof(charTable) / sizeof(GattCharacteristic *));

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

/*  Replacement for Handle Value Notifications. Block read and write is initiated by the client.
    This function is only for sending short messages such as update notifications.
*/
ble_error_t BlockTransferService::updateCharacteristicValue(const uint8_t *value, uint16_t size)
{
    ble_error_t returnValue = BLE_ERROR_BUFFER_OVERFLOW;

    // only messages that fit a single packet are sent
    if (size <= MAX_DIRECT_READ_PAYLOAD_SIZE)
    {
        // insert header
        uint8_t payload[DIRECT_READ_HEADER_SIZE + size];
        payload[0] = BT_TYPE_READ_NOTIFY;

        // insert payload
        memcpy(&payload[1], value, size);

        // send
        returnValue = ble.updateCharacteristicValue(readFromHandle, payload, DIRECT_READ_HEADER_SIZE + size);
    }

    return returnValue;
}


/*  Client send a read request. Pass request to callback function which updates the read buffer block.
    Based on the length the request is:
    1. denied
    2. send as a direct message
    3. send as a block transfer
*/
void BlockTransferService::onReadRequest(GattCharacteristicReadAuthCBParams* event)
{
    BLE_DEBUG("read request\n\r");

    if (event->charHandle == readFromHandle)
    {
        readBlock->offset = event->offset;

        /* call higher layer for read value */
        readRequestHandler(readBlock);

        // read denied
        if (readBlock->length == 0)
        {
            event->authorizationReply = AUTH_CALLBACK_REPLY_ATTERR_READ_NOT_PERMITTED;
        }
        // send the message directly if it can fit in one packet
        else if (readBlock->length < 20)
        {
            directBlock[0] = BT_TYPE_READ_DIRECT;

            memcpy(&directBlock[1], readBlock->data, readBlock->length);

            event->authorizationReply = AUTH_CALLBACK_REPLY_SUCCESS;
            event->data = directBlock;
            event->len = readBlock->length + 1;
        }
        else
        {
            // guard against retransmission received in the wrong state
            if (blockTransferState == BT_STATE_OFF)
            {
                /*  The block is too large to fit in one packet.
                    Respond with a setup packet specifying the number of fragments available.
                */
                readTotalFragments = readBlock->length / MAX_BLOCK_PAYLOAD_SIZE;

                if (readTotalFragments * MAX_BLOCK_PAYLOAD_SIZE < readBlock->length)
                {
                    readTotalFragments++;
                }

                // update internal state
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

            event->authorizationReply = AUTH_CALLBACK_REPLY_SUCCESS;
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
    if (event->charHandle == writeToHandle)
    {
        const uint8_t* message = event->data;
        const bt_type_t messageType = (bt_type_t) message[0];

        switch(messageType)
        {
            case BT_TYPE_WRITE_SETUP:
                {
                    // guard against retransmissions resetting the state
                    if (blockTransferState == BT_STATE_OFF)
                    {
                        /*  Write setup message received.
                        */

                        // reset receive buffer to full length
                        writeBlock->length = writeBlock->maxLength;
                        writeBlock->offset = 0;

                        // block length, number is LSB
                        uint16_t blockLength;
                        blockLength = message[2];
                        blockLength = (blockLength << 8) | message[1];

                        // check if the write request fits in memory
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
                            /*  Set the size of the current block to 0. This will trigger the
                                requestMissing call to send an "all fragments received" message.
                                This tells the client that the requested write was too large.
                            */
                            setIndexSetSize(&receiveBlockMissingFragments, 0);
                        }
                    }

                    /*  Acknowledge setup by requesting data or signal that the request was too large.
                    */
                    requestMissing();

                    BLE_DEBUG("setup fragments (%d) (%d)\r\n", receiveBlockTotalFragments, event->len);
                }

                break;

            case BT_TYPE_WRITE_DIRECT:
                {
                    /*  Direct write message received.
                        Send payload to upper layer.
                    */

                    // the offset of the current block with regards to the overall characteristic
                    receiveBlockOffset = message[2];
                    receiveBlockOffset = (receiveBlockOffset << 8) | message[1];

                    // discard header, grab payload
                    memcpy(writeBlock->data, &event->data[3], event->len - 3);
                    writeBlock->offset = receiveBlockOffset;
                    writeBlock->length = event->len - 3;

                    /*  Full block received. No change in state.
                        Signal upper layer of write request.
                    */
                    writeBlock = writeDoneHandler(writeBlock);
                }
                break;

            case BT_TYPE_WRITE_PAYLOAD_MORE:
            case BT_TYPE_WRITE_PAYLOAD_LAST:
                {
                    if (blockTransferState == BT_STATE_SERVER_WRITE)
                    {
                        /*  Write fragment received and it was expected
                        */

                        uint16_t fragmentNumber;

                        // number is LSB
                        fragmentNumber = message[2];
                        fragmentNumber = (fragmentNumber << 8) | message[1];

                        // check if fragment is a duplicate
                        if (containsIndex(&receiveBlockMissingFragments, fragmentNumber))
                        {
                            BLE_DEBUG("%d\r\n", fragmentNumber);

                            // mark fragment as received in the index
                            removeIndex(&receiveBlockMissingFragments, fragmentNumber);

                            // copy payload to receive buffer
                            uint16_t index = fragmentNumber * MAX_BLOCK_PAYLOAD_SIZE;
                            memcpy(&writeBlock->data[index], &event->data[3], event->len - 3);

                            /*  When sender signals "no more data", request missing fragments
                                or signal reception complete.
                            */
                            if (messageType == BT_TYPE_WRITE_PAYLOAD_LAST)
                            {
                                if (receiveBlockMissingFragments.count == 0)
                                {
                                    // signal upper layer of write request
                                    writeBlock = writeDoneHandler(writeBlock);

                                    // reset state
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
                    /*  Read fragment request.
                    */

                    uint16_t sendFromFragment;
                    uint16_t sendAmount;

                    // fragments in block, number is send LSB
                    sendFromFragment = message[2];
                    sendFromFragment = (sendFromFragment << 8) | message[1];

                    // the offset of the current block with regards to the overall characteristic
                    sendAmount = message[4];
                    sendAmount = (sendAmount << 8) | message[3];

                    /*  If the request is within bounds it is a genuine request for fragments.
                        Otherwise, treat it as an "all done" message.
                    */
                    if (sendFromFragment < readTotalFragments)
                    {
                        readFragmentIndex = sendFromFragment;
                        readFragmentsInBatch = sendAmount;

                        BLE_DEBUG("read request %d %d %d\r\n", sendFromFragment, sendAmount, readTotalFragments);

                        // send requested fragments
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

/*  Send missing fragments in current batch. For each successful transmission
    update variables to point to the next fragment.
*/
void BlockTransferService::sendReadReply(void)
{
    BLE_DEBUG("send read reply\r\n");

    // while there are still fragments in this batch
    while (readFragmentsInBatch != 0)
    {
        // send current fragment
        bool result = sendReadReplyRepeatedly();

        BLE_DEBUG("notify: %d %d\r\n", readFragmentIndex, result);

        // update to next fragment if successful
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

/* Send the current fragment in this batch.
*/
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

    // insert payload
    uint16_t index = readFragmentIndex * MAX_BLOCK_PAYLOAD_SIZE;
    memcpy(&payload[3], &readBlock->data[index], length);

    // try to send fragment
    ble_error_t didSendValue = ble.updateCharacteristicValue(readFromHandle, payload, BLOCK_HEADER_SIZE + length);

    return (didSendValue == BLE_ERROR_NONE);
}

/*  Function for requesting write fragments or signal reception complete.
*/
void BlockTransferService::requestMissing(void)
{
    BLE_DEBUG("request missing\r\n");

    // Check IndexSet if there are still missing fragments in the current block
    if (receiveBlockMissingFragments.count > 0)
    {
        // find missing ranges
        uint16_t fragmentNumber;
        uint16_t count;

        findMissing(&receiveBlockMissingFragments, &fragmentNumber, &count);

        // construct write request
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
