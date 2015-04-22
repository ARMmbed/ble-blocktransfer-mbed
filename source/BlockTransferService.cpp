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

#include "ble-blocktransfer/BlockTransferService.h"

#if 0
#define BLE_DEBUG(...) { printf(__VA_ARGS__); }
#else
#define BLE_DEBUG(...) /* nothing */
#endif

const uint16_t ServiceWriteCharacteristicShortUUID = 0x0001;
const uint16_t ServiceReadCharacteristicShortUUID  = 0x0002;

BlockTransferService::BlockTransferService(BLEDevice &_ble, const UUID &uuid)
    :   ble(_ble),
        readRequestHandler(),
        writeDoneHandler(NULL, NULL),
        writeBlock(NULL),
        receiveBuffer(),
        sendBuffer(),
        readBlock(&readBlockData),
        readBlockData(),
        receiveLengthOffset(0),
        receiveFragmentOffset(0),
        receiveTotalFragments(0),
        currentMTU(BTS_MTU_SIZE_DEFAULT),
        maxBlockPayloadSize(BTS_MTU_SIZE_DEFAULT - BLOCK_HEADER_SIZE),
        maxDirectReadPayloadSize(BTS_MTU_SIZE_DEFAULT - DIRECT_READ_HEADER_SIZE),
        internalState(BT_STATE_OFF)
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

    // initialize bitmap for keeping track of fragments
    initIndexSet(&receiveBlockMissingFragments, indexBuffer, 30);
}

/*  Set "write received" callback function and write buffer.

    The function is called when a block of data has been written to the write buffer.
    The callback can either return the same block or swap it with a different one.
*/
void BlockTransferService::setWriteAuthorizationCallback(block_t* (*writeHandler)(block_t*), block_t* _writeBlock)
{
    /*  Guard against resetting callback and write block while in the middle of a transfer. */
    if (internalState == BT_STATE_OFF)
    {
        writeDoneHandler.attach(writeHandler);
        writeBlock = _writeBlock;
    }
}

/*  Set "read requested" callback function.

    The function is called when a client wants to read the Block Transfer service.
    The function can modify the block_t fields 'uint8_t* data' and 'uint32_t length'
    to return the data to be sent back to the client.

    Setting the length to '0' means the read request was denied.
*/
void BlockTransferService::setReadAuthorizationCallback(void (*readHandler)(block_t*))
{
    /*  Guard against resetting callback while in the middle of a transfer. */
    if (internalState == BT_STATE_OFF)
    {
        readRequestHandler.attach(readHandler);
    }
}

/*  Replacement for Handle Value Notifications. Block read and write is initiated by the client.
    This function is only for sending short messages such as update notifications.
*/
ble_error_t BlockTransferService::updateCharacteristicValue(const uint8_t *value, uint16_t size)
{
    ble_error_t returnValue = BLE_ERROR_BUFFER_OVERFLOW;

    // only messages that fit a single packet are sent
    if (size <= maxDirectReadPayloadSize)
    {
        uint16_t payloadLength = DIRECT_READ_HEADER_SIZE + size;

        // insert header
        uint8_t payload[payloadLength];
        payload[0] = BT_TYPE_READ_NOTIFY << 4;

        // insert payload
        memcpy(&payload[1], value, size);

        // send
        returnValue = ble.updateCharacteristicValue(readFromHandle, payload, payloadLength);
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
    BLE_DEBUG("read: event: %d\r\n", internalState);

    if (event->charHandle == readFromHandle)
    {
        readBlock->offset = event->offset;

        /* call higher layer for read value */
        readRequestHandler.call(readBlock);

        // read denied
        if (readBlock->length == 0)
        {
            event->authorizationReply = AUTH_CALLBACK_REPLY_ATTERR_READ_NOT_PERMITTED;
        }
        // send the message directly if it can fit in one packet
        else if (readBlock->length <= maxDirectReadPayloadSize)
        {
            directBlock[0] = BT_TYPE_READ_DIRECT << 4;

            memcpy(&directBlock[1], readBlock->data, readBlock->length);

            event->authorizationReply = AUTH_CALLBACK_REPLY_SUCCESS;
            event->data = directBlock;
            event->len = readBlock->length + 1;
        }
        else
        {
            // guard against retransmission received in the wrong state
            if (internalState == BT_STATE_OFF)
            {
                /*  The block is too large to fit in one packet.
                    Respond with a setup packet specifying the number of fragments available.
                */
                readTotalFragments = readBlock->length / maxBlockPayloadSize;

                BLE_DEBUG("read: setup: %d %d %d\r\n", readBlock->length, readTotalFragments, maxBlockPayloadSize);

                if (readTotalFragments * maxBlockPayloadSize < readBlock->length)
                {
                    readTotalFragments++;
                }

                // update internal state
                internalState = BT_STATE_SERVER_READ;

                /*  Respond with setup message.
                    When the receiver is ready for data it will request fragments.
                */
                directBlock[0] = BT_TYPE_READ_SETUP << 4;
                directBlock[1] = readBlock->length;
                directBlock[2] = readBlock->length >> 8;
                directBlock[3] = readBlock->length >> 16;
                directBlock[4] = readTotalFragments;
                directBlock[5] = readTotalFragments >> 8;
                directBlock[6] = readTotalFragments >> 16;

                event->authorizationReply = AUTH_CALLBACK_REPLY_SUCCESS;
                event->data = directBlock;
                event->len = 7;
            }
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
        const bt_type_t messageType = (bt_type_t) (message[0] >> 4);

        BLE_DEBUG("onDataWritten: ");
#if 0

        for (int idx = 0; idx < event->len; idx++)
        {
            BLE_DEBUG("%02X", event->data[idx]);
        }
#endif
        BLE_DEBUG("\r\n");

        switch(messageType)
        {
            case BT_TYPE_WRITE_SETUP:
                {
                    // guard against retransmissions resetting the state
                    if (internalState == BT_STATE_OFF)
                    {
                        /*  Write setup message received.
                        */

                        /*  Guard against write block being NULL. */
                        uint32_t maxLength = 0;

                        if (writeBlock)
                        {
                            maxLength = writeBlock->maxLength;
                        }

                        BLE_DEBUG("write: maxLength: %d\r\n", maxLength);

                        if (maxLength < maxBlockPayloadSize)
                        {
                            /*  Buffer is not large enough to hold a single fragment.
                                Signal client that transfer is done. The client knows
                                something is wrong because no fragments have been sent.
                            */
                            setIndexSetSize(&receiveBlockMissingFragments, 0);
                        }
                        else
                        {
                            // block length, number is LSB
    //                        receiveTotalLength = message[3];
    //                        receiveTotalLength = (receiveTotalLength << 8) | message[2];
    //                        receiveTotalLength = (receiveTotalLength << 8) | message[1];

                            // the offset of the current block with regards to the overall characteristic
                            receiveLengthOffset = message[6];
                            receiveLengthOffset = (receiveLengthOffset << 8) | message[5];
                            receiveLengthOffset = (receiveLengthOffset << 8) | message[4];

                            // reset relation between absolute and relative fragment number
                            receiveFragmentOffset = 0;

                            // fragments in block, number is send LSB
                            receiveTotalFragments = message[9];
                            receiveTotalFragments = (receiveTotalFragments << 8) | message[8];
                            receiveTotalFragments = (receiveTotalFragments << 8) | message[7];

                            // find max fragments in writeBlock
                            uint32_t maxFragments = maxLength / maxBlockPayloadSize;

                            // check if the remaining fragments fit in a single batch transfer
                            uint32_t fragments = (maxFragments < receiveTotalFragments)
                                                ? maxFragments : receiveTotalFragments;

                            // use IndexSet to keep track of the received fragments and find those missing
                            setIndexSetSize(&receiveBlockMissingFragments, fragments);

                            internalState = BT_STATE_SERVER_WRITE;

                            BLE_DEBUG("write: setup fragments (%d) (%d)\r\n", receiveTotalFragments, maxBlockPayloadSize);
                        }
                    }

                    /*  Acknowledge setup by requesting data.
                    */
                    requestMissing();
                }

                break;

            case BT_TYPE_WRITE_DIRECT:
                {
                    /*  Direct write message received.
                        Send payload to upper layer.
                    */

                    /*  Guard against write block being NULL. */
                    if (writeBlock)
                    {
                        // the offset of the current block with regards to the overall characteristic
                        uint32_t offset;
                        offset = message[2];
                        offset = (offset << 8) | message[1];
                        offset = (offset << 4) | (message[0] & 0x0F);
                        writeBlock->offset = offset;

                        // payload length
                        uint16_t currentPayloadLength = event->len - DIRECT_WRITE_HEADER_SIZE;
                        writeBlock->length = currentPayloadLength;

                        // payload
                        memcpy(writeBlock->data, &event->data[3], currentPayloadLength);

                        /*  Full block received. No change in state.
                            Signal upper layer of write request.
                        */
                        writeBlock = writeDoneHandler.call(writeBlock);
                    }
                }
                break;

            case BT_TYPE_WRITE_PAYLOAD_MORE:
            case BT_TYPE_WRITE_PAYLOAD_LAST:
                {
                    if (internalState == BT_STATE_SERVER_WRITE)
                    {
                        /*  Write fragment received and it was expected
                        */

                        uint32_t absoluteFragmentIndex;
                        absoluteFragmentIndex = message[2];
                        absoluteFragmentIndex = (absoluteFragmentIndex << 8) | message[1];
                        absoluteFragmentIndex = (absoluteFragmentIndex << 4) | (message[0] & 0xF);

                        // subtract fragment offset to get number relative to the current transfer
                        uint16_t relativeFragmentIndex = absoluteFragmentIndex - receiveFragmentOffset;

                        // check if fragment is a duplicate
                        if (containsIndex(&receiveBlockMissingFragments, relativeFragmentIndex))
                        {
                            BLE_DEBUG("write: fragment: %d %d : ", absoluteFragmentIndex, relativeFragmentIndex);
#if 0
                            for (int idx = 0; idx < event->len; idx++)
                            {
                                BLE_DEBUG("%02X", event->data[idx]);
                            }
#endif
                            BLE_DEBUG("\r\n");

                            // mark fragment as received in the index
                            removeIndex(&receiveBlockMissingFragments, relativeFragmentIndex);

                            // copy payload to receive buffer
                            uint16_t processedLength = relativeFragmentIndex * maxBlockPayloadSize;
                            uint16_t currentPayloadLength = event->len - BLOCK_HEADER_SIZE;
                            memcpy(&writeBlock->data[processedLength], &event->data[3], currentPayloadLength);

                            BLE_DEBUG("%d\r\n", processedLength);

                            /*  When sender signals "no more data", request missing fragments
                                or signal reception complete.
                            */
                            if (messageType == BT_TYPE_WRITE_PAYLOAD_LAST)
                            {
                                if (receiveBlockMissingFragments.count == 0)
                                {
                                    // update block offset and length
                                    writeBlock->offset = receiveLengthOffset;
                                    writeBlock->length = processedLength + currentPayloadLength;

                                    // signal upper layer of write request
                                    writeBlock = writeDoneHandler.call(writeBlock);

                                    /*  If this is a multi-batch transfer update
                                        index set to reflect the missing fragments,
                                        otherwise reset internal state.
                                    */
                                    if ((absoluteFragmentIndex + 1) < receiveTotalFragments)
                                    {
                                        // update length offset
                                        receiveLengthOffset += processedLength + currentPayloadLength;

                                        // update fragment offset
                                        receiveFragmentOffset = absoluteFragmentIndex + 1;

                                        // find max fragments in new writeBlock
                                        uint16_t maxFragments = writeBlock->maxLength / maxBlockPayloadSize;

                                        // total remaining fragments
                                        uint16_t remainingFragments = receiveTotalFragments - (absoluteFragmentIndex + 1);

                                        // check if the remaining fragments fit in a single batch transfer
                                        uint16_t fragments = (maxFragments < remainingFragments)
                                                            ? maxFragments : remainingFragments;

                                        // use IndexSet to keep track of the received fragments and find those missing
                                        setIndexSetSize(&receiveBlockMissingFragments, fragments);

                                        BLE_DEBUG("write: next batch: %d %d\r\n", receiveFragmentOffset, receiveTotalFragments);
                                    }
                                    else
                                    {
                                        // all done, reset state
                                        internalState = BT_STATE_OFF;
                                    }
                                }

                                // request missing fragments or acknowledge the reception of the last fragment
                                requestMissing();
                            }
                        }
                        else
                        {
                            BLE_DEBUG("write: duplicate %d\r\n", relativeFragmentIndex);
                        }
                    }
                }
                break;

            case BT_TYPE_READ_REQUEST:
                {
                    /*  Read fragment request.
                    */

                    uint32_t sendFromFragment;
                    uint32_t sendAmount;

                    // fragments in block, number is send LSB
                    sendFromFragment = message[3];
                    sendFromFragment = (sendFromFragment << 8) | message[2];
                    sendFromFragment = (sendFromFragment << 8) | message[1];

                    // the offset of the current block with regards to the overall characteristic
                    sendAmount = message[6];
                    sendAmount = (sendAmount << 8) | message[5];
                    sendAmount = (sendAmount << 8) | message[4];

                    /*  If the request is within bounds it is a genuine request for fragments.
                        Otherwise, treat it as an "all done" message.
                    */
                    if (sendFromFragment < readTotalFragments)
                    {
                        readFragmentIndex = sendFromFragment;
                        readFragmentsInBatch = sendAmount;

                        BLE_DEBUG("read: request %d %d %d\r\n", sendFromFragment, sendAmount, readTotalFragments);

                        // send requested fragments
                        sendReadReply();
                    }
                    else
                    {
                        internalState = BT_STATE_OFF;
                        BLE_DEBUG("read: complete\r\n");
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

    internalState = BT_STATE_OFF;
}

/*  This function is called when the BLE device is ready for more characteristic value updates
    and is shared by all characteristics.
*/
void BlockTransferService::onDataSent(unsigned count)
{
    BLE_DEBUG("hvx sent: %d\r\n", count);

    if (internalState == BT_STATE_SERVER_READ)
    {
        sendReadReply();
    }
}

/*  Send missing fragments in current batch. For each successful transmission
    update variables to point to the next fragment.
*/
void BlockTransferService::sendReadReply(void)
{
    BLE_DEBUG("read: reply\r\n");

    // while there are still fragments in this batch
    while (readFragmentsInBatch != 0)
    {
        // send current fragment
        bool result = sendReadReplyRepeatedly();

        BLE_DEBUG("read: notify: %d %d\r\n", readFragmentIndex, result);

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
    // data already sent
    uint16_t processedLength = readFragmentIndex * maxBlockPayloadSize;

    // find length. the last packet might be shorter.
    // readFragmentIndex is zero-indexed
    uint16_t payloadLength = (readFragmentIndex < (readTotalFragments - 1))
                            ? maxBlockPayloadSize
                            : readBlock->length - processedLength;

    // set data type based on whether more data is pending in this batch
    uint8_t type = (readFragmentsInBatch == 1) ? BT_TYPE_READ_PAYLOAD_LAST : BT_TYPE_READ_PAYLOAD_MORE;

    // insert header
    uint8_t payload[BLOCK_HEADER_SIZE + payloadLength];
    payload[0] = (type << 4) | (readFragmentIndex & 0x0F);
    payload[1] = readFragmentIndex >> 4;
    payload[2] = readFragmentIndex >> 12;

    // insert payload
    memcpy(&payload[3], &readBlock->data[processedLength], payloadLength);

    // try to send fragment
    ble_error_t didSendValue = ble.updateCharacteristicValue(readFromHandle, payload, BLOCK_HEADER_SIZE + payloadLength);

    return (didSendValue == BLE_ERROR_NONE);
}

/*  Function for requesting write fragments or signal reception complete.
*/
void BlockTransferService::requestMissing(void)
{
    BLE_DEBUG("write: missing\r\n");

    // Check IndexSet if there are still missing fragments in the current block
    if (receiveBlockMissingFragments.count > 0)
    {
        // find missing ranges
        uint16_t relativeFragmentIndex;
        uint16_t count;

        findMissing(&receiveBlockMissingFragments, &relativeFragmentIndex, &count);

        // add the fragment offset to get the absolute fragment number
        uint16_t absoluteFragmentIndex = relativeFragmentIndex + receiveFragmentOffset;

        // construct write request
        uint8_t request[7];

        request[0] = BT_TYPE_WRITE_REQUEST << 4;
        request[1] = absoluteFragmentIndex;
        request[2] = absoluteFragmentIndex >> 8;
        request[3] = absoluteFragmentIndex >> 16;
        request[4] = count;
        request[5] = count >> 8;
        request[6] = count >> 16;

        ble.updateCharacteristicValue(readFromHandle, request, 7);

        BLE_DEBUG("write: send req: %d %d\r\n", absoluteFragmentIndex, count);
    }
    else
    {
        // send acknowledgment for last fragment
        uint8_t request[7];

        request[0] = (BT_TYPE_WRITE_REQUEST << 4);
        request[1] = 0xFF;
        request[2] = 0xFF;
        request[3] = 0xFF;
        request[4] = 0x01;
        request[5] = 0x00;
        request[6] = 0x00;

        ble.updateCharacteristicValue(readFromHandle, request, 7);

        BLE_DEBUG("write: send ack: 0xFFFFFF 1\r\n");
    }
}
