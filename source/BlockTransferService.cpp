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
// increase timeout
#undef FRAGMENT_TIMEOUT_US
#define FRAGMENT_TIMEOUT_US (1000*1000)
#define BLE_DEBUG(...) { printf(__VA_ARGS__); }
#else
#define BLE_DEBUG(...) /* nothing */
#endif

#define BTS_MAX_BLOCK_SIZE 1024
#define BTS_INVALID_HANDLE 0xFFFF

const uint16_t ServiceWriteCharacteristicShortUUID = 0x0001;
const uint16_t ServiceReadCharacteristicShortUUID  = 0x0002;

BlockTransferService::BlockTransferService()
    :   ble(),
        connectionHandle(BTS_INVALID_HANDLE),
        connectionCounter(0),
        readRequestHandler(),
        writeDoneHandler(),
        readDoneHandler(),
        writeBlock(),
        readFromCharacteristic(ServiceReadCharacteristicShortUUID,
                               sendBuffer, 1, BTS_MTU_SIZE_DEFAULT,
                               GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ |
                               GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY),
        writeToCharacteristic(ServiceWriteCharacteristicShortUUID,
                              receiveBuffer, 1, BTS_MTU_SIZE_DEFAULT,
                              GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE_WITHOUT_RESPONSE),
        readBlock(NULL),
        receiveLengthOffset(0),
        receiveFragmentOffset(0),
        receiveTotalFragments(0),
        currentMTU(BTS_MTU_SIZE_DEFAULT),
        maxBlockPayloadSize(BTS_MTU_SIZE_DEFAULT - BLOCK_HEADER_SIZE),
        maxDirectReadPayloadSize(BTS_MTU_SIZE_DEFAULT - DIRECT_READ_HEADER_SIZE),
        readState(BT_STATE_OFF),
        writeState(BT_STATE_OFF)
{}

void BlockTransferService::init(const UUID& uuid,
                                SecurityManager::SecurityMode_t securityMode)
{
    ble.init();

    /*  Enable security.
    */
    ble.securityManager().init();
    readFromCharacteristic.requireSecurity(securityMode);
//    writeToCharacteristic->requireSecurity(securityMode);

    /*  Register callback.
    */
    readFromCharacteristic.setReadAuthorizationCallback(this, &BlockTransferService::onReadRequest);

    /*  Setup standard BLE read and write characteristics on which the block transfer protocol is built upon.
    */
    GattCharacteristic *charTable[] = {&readFromCharacteristic, &writeToCharacteristic};
    GattService BTService(uuid, charTable, sizeof(charTable) / sizeof(GattCharacteristic *));

    ble.gattServer().addService(BTService);

    // handles are set when the service has been added to the ble device
    readFromHandle = readFromCharacteristic.getValueHandle();
    writeToHandle = writeToCharacteristic.getValueHandle();

    // register callback functions
    ble.gattServer().onDataWritten(this, &BlockTransferService::onDataWritten);
    ble.gattServer().onDataSent(this, &BlockTransferService::onDataSent);
    ble.gap().onConnection(this, &BlockTransferService::onConnection);
    ble.gap().onDisconnection(this, &BlockTransferService::onDisconnection);
}

/*  Set "write received" callback function and write buffer.

    The function is called when a block of data has been written to the write buffer.
*/
void BlockTransferService::setWriteAuthorizationCallback(void (*writeHandler)(SharedPointer<Block>))
{
    /*  Guard against resetting callback and write block while in the middle of a transfer. */
    if ((writeState == BT_STATE_OFF) || (writeState == BT_STATE_READY))
    {
        writeDoneHandler.attach(writeHandler);
    }
}

/*  Set "read requested" callback function.

    The function is called when a client wants to read the Block Transfer service.
    The function can modify the Block fields 'uint8_t* data' and 'uint32_t length'
    to return the data to be sent back to the client.

    Setting the length to '0' means the read request was denied.
*/
void BlockTransferService::setReadAuthorizationCallback(SharedPointer<Block> (*readHandler)(uint32_t))
{
    /*  Guard against resetting callback while in the middle of a transfer. */
    if ((readState == BT_STATE_OFF) || (readState == BT_STATE_READY))
    {
        readRequestHandler.attach(readHandler);
    }
}

void BlockTransferService::setReadDoneCallback(void (*readHandler)(void))
{
    /*  Guard against resetting callback while in the middle of a transfer. */
    if ((readState == BT_STATE_OFF) || (readState == BT_STATE_READY))
    {
        readDoneHandler.attach(readHandler);
    }
}

/*  Replacement for Handle Value Notifications. Block read and write is initiated by the client.
    This function is only for sending short messages such as update notifications.
*/
ble_error_t BlockTransferService::updateCharacteristicValue(const uint8_t* value, uint16_t size)
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
        memcpy(&(payload[1]), value, size);

        // send
        returnValue = ble.gattServer().write(connectionHandle, readFromHandle, payload, payloadLength);
    }

    return returnValue;
}

bool BlockTransferService::writeInProgess(void)
{
    return ((writeState == BT_STATE_SERVER_WRITE) || (writeState == BT_STATE_SERVER_WRITE_ACK));
}

bool BlockTransferService::readInProgress(void)
{
    return (readState == BT_STATE_SERVER_READ);
}

bool BlockTransferService::isReady(void)
{
    return ((writeState == BT_STATE_READY) && (readState == BT_STATE_READY));
}

/*****************************************************************************/
/* Events                                                                    */
/*****************************************************************************/

/*  Client send a read request. Pass request to callback function which updates the read buffer block.
    Based on the length the request is:
    1. denied
    2. send as a direct message
    3. send as a block transfer
*/
void BlockTransferService::onReadRequest(GattReadAuthCallbackParams* event)
{
    BLE_DEBUG("bts: read event: %d\r\n", readState);

    /*  only respond to requests directed to this Block Transfer handle
    */
    if (event->handle == readFromHandle)
    {
        // if busy servicing another read, respond with a read not permitted
        if ((readState != BT_STATE_READY) ||
            // or if busy on a different connection
            ((connectionHandle != BTS_INVALID_HANDLE) && (connectionHandle != event->connHandle)))
        {
            event->authorizationReply = AUTH_CALLBACK_REPLY_ATTERR_READ_NOT_PERMITTED;
        }
        else
        {
            /* call higher layer for read value */
            readBlock = readRequestHandler.call(event->offset);

            // check return pointer is valid
            if (readBlock == NULL)
            {
                event->authorizationReply = AUTH_CALLBACK_REPLY_ATTERR_READ_NOT_PERMITTED;
            }
            else
            {
                // read denied
                if (readBlock->getLength() == 0)
                {
                    event->authorizationReply = AUTH_CALLBACK_REPLY_ATTERR_READ_NOT_PERMITTED;
                }
                // send the message directly if it can fit in one packet
                else if (readBlock->getLength() <= maxDirectReadPayloadSize)
                {
                    directBlock[0] = BT_TYPE_READ_DIRECT << 4;

                    readBlock->memcpy(&(directBlock[1]), 0, readBlock->getLength());

                    event->authorizationReply = AUTH_CALLBACK_REPLY_SUCCESS;
                    event->data = directBlock;
                    event->len = readBlock->getLength() + 1;
                }
                else
                {
                    /*  The block is too large to fit in one packet.
                        Respond with a setup packet specifying the number of fragments available.
                    */
                    readTotalFragments = (readBlock->getLength() + (maxBlockPayloadSize - 1)) / maxBlockPayloadSize;

                    BLE_DEBUG("bts: read: setup: %d %d %d\r\n", readBlock->getLength(), readTotalFragments, maxBlockPayloadSize);

                    // update read state and connection handle
                    readState = BT_STATE_SERVER_READ;
                    connectionHandle = event->connHandle;

                    /*  Respond with setup message.
                        When the receiver is ready for data it will request fragments.
                    */
                    directBlock[0] = BT_TYPE_READ_SETUP << 4;
                    directBlock[1] = readBlock->getLength();
                    directBlock[2] = readBlock->getLength() >> 8;
                    directBlock[3] = readBlock->getLength() >> 16;
                    directBlock[4] = readTotalFragments;
                    directBlock[5] = readTotalFragments >> 8;
                    directBlock[6] = readTotalFragments >> 16;

                    event->authorizationReply = AUTH_CALLBACK_REPLY_SUCCESS;
                    event->data = directBlock;
                    event->len = 7;
                }
            } // end "is readBlock NULL"
        } // end "is the server busy"
    } // end "is this the correct gatt handle"
}


/*  This function is called when any writes have been received by the BLE device.
    Must filter on characteristic handle to ensure we only respond to writes
    intended for this characteristic.
*/
void BlockTransferService::onDataWritten(const GattWriteCallbackParams* event)
{
    if (event->handle == writeToHandle)
    {
#if 0
        BLE_DEBUG("bts: onDataWritten: ");
        for (int idx = 0; idx < event->len; idx++)
        {
            BLE_DEBUG("%02X", event->data[idx]);
        }
        BLE_DEBUG("\r\n");
#endif

        // only accept new writes from connections already being served
        // or when connectionHandle is NULL
        if ((connectionHandle == BTS_INVALID_HANDLE) || (event->connHandle == connectionHandle))
        {
            const uint8_t* message = event->data;
            const bt_type_t messageType = (bt_type_t) (message[0] >> 4);

            switch(messageType)
            {
               case BT_TYPE_WRITE_SETUP:
                    {
                        // guard against retransmissions resetting the state
                        if (writeState == BT_STATE_READY)
                        {
                            /*  Write setup message received.
                            */

                            // block length, number is LSB
                            uint32_t receiveTotalLength;

                            receiveTotalLength = message[3];
                            receiveTotalLength = (receiveTotalLength << 8) | message[2];
                            receiveTotalLength = (receiveTotalLength << 8) | message[1];

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

                            // allocate memory
                            uint8_t* buffer = NULL;
                            uint32_t bufferLength = 0;

                            // use the fragement count directly if the block fits
                            if (receiveTotalLength <= MAX_BLOCK_TRANSFER_SIZE)
                            {
                                buffer = (uint8_t*) malloc(receiveTotalLength);
                                bufferLength = receiveTotalLength;

                                // use IndexSet to keep track of the received fragments and find those missing
                                missingFragments.setSize(receiveTotalFragments);
                            }
                            else
                            {
                                buffer = (uint8_t*) malloc(MAX_BLOCK_TRANSFER_SIZE);
                                bufferLength = MAX_BLOCK_TRANSFER_SIZE;

                                // find the maximum number of fragments that fits the block
                                // Note: this calculation is rounding down
                                missingFragments.setSize(MAX_BLOCK_TRANSFER_SIZE / maxBlockPayloadSize);
                            }

                            // accept write request if memory allocation was successful
                            if (buffer)
                            {
                                // allocate reference counted dynamic memory buffer
                                writeBlock = SharedPointer<Block>(new BlockDynamic(buffer, bufferLength));

                                // enter write state and store connection handle
                                writeState = BT_STATE_SERVER_WRITE;
                                connectionHandle = event->connHandle;

                                BLE_DEBUG("bts: write setup fragments: %d (%d) (%d)\r\n", connectionHandle, receiveTotalFragments, maxBlockPayloadSize);
                            }
                            else
                            {
                                /*  Buffer could not be allocated.
                                    Signal client that transfer is done. The client knows
                                    something is wrong because no fragments have been sent.
                                */
                                missingFragments.setSize(0);
                            }
                        }

                        /*  Acknowledge setup by requesting data.
                        */
                        sendWriteRequestMissing();
                    }

                    break;

                case BT_TYPE_WRITE_DIRECT:
                    {
                        /*  Direct write message received.
                            Send payload to upper layer.
                        */

                        // payload length
                        uint16_t payloadLength = event->len - DIRECT_WRITE_HEADER_SIZE;
                        uint8_t* buffer = (uint8_t*) malloc(payloadLength);

                        // only continue if allocation was successful
                        if (buffer)
                        {
                            // allocate reference counted dynamic memory buffer
                            SharedPointer<Block> block(new BlockDynamic(buffer, payloadLength));

                            // set offset of the current block with regards to the overall characteristic
                            uint32_t offset;
                            offset = message[2];
                            offset = (offset << 8) | message[1];
                            offset = (offset << 4) | (message[0] & 0x0F);
                            block->setOffset(offset);

                            // copy payload
                            block->memcpy(0, &(event->data[3]), payloadLength);

                            /*  Full block received. No change in state.
                                Signal upper lay er of write request.
                            */
                            writeDoneHandler.call(block);
                        }
                    }
                    break;

                case BT_TYPE_WRITE_PAYLOAD_MORE:
                case BT_TYPE_WRITE_PAYLOAD_LAST:
                    {
                        if (writeState == BT_STATE_SERVER_WRITE)
                        {
                            // fragment received, reset timeout
                            timeout.attach_us(this, &BlockTransferService::fragmentTimeout, FRAGMENT_TIMEOUT_US);

                            /*  Write fragment received and it was expected
                            */

                            uint32_t absoluteFragmentIndex;
                            absoluteFragmentIndex = message[2];
                            absoluteFragmentIndex = (absoluteFragmentIndex << 8) | message[1];
                            absoluteFragmentIndex = (absoluteFragmentIndex << 4) | (message[0] & 0xF);

                            // subtract fragment offset to get number relative to the current transfer
                            uint16_t relativeFragmentIndex = absoluteFragmentIndex - receiveFragmentOffset;

                            // check if fragment is a duplicate
                            if (missingFragments.containsIndex(relativeFragmentIndex))
                            {
                                BLE_DEBUG("\t: %5d %3d : ", absoluteFragmentIndex, relativeFragmentIndex);
#if 0
                                for (int idx = 0; idx < event->len; idx++)
                                {
                                    BLE_DEBUG("%02X", event->data[idx]);
                                }
                                BLE_DEBUG("\r\n");
#endif

                                // mark fragment as received in the index
                                missingFragments.removeIndex(relativeFragmentIndex);

                                // copy payload to receive buffer
                                uint16_t processedLength = relativeFragmentIndex * maxBlockPayloadSize;
                                uint16_t currentPayloadLength = event->len - BLOCK_HEADER_SIZE;

                                writeBlock->memcpy(processedLength, &(event->data[3]), currentPayloadLength);

                                BLE_DEBUG("%4d\r\n", processedLength);

                                /*  When sender signals "no more data", request missing fragments
                                    or signal reception complete.
                                */
                                if (messageType == BT_TYPE_WRITE_PAYLOAD_LAST)
                                {
                                    // last fragment received, cancel timeout
                                    timeout.detach();

                                    if (missingFragments.getCount() == 0)
                                    {
                                        // update block offset and length
                                        writeBlock->setOffset(receiveLengthOffset);
                                        writeBlock->setLength(processedLength + currentPayloadLength);

                                        /*  If this is a multi-batch transfer update
                                            index set to reflect the missing fragments,
                                            and call writeDoneHandler.
                                        */
                                        if ((absoluteFragmentIndex + 1) < receiveTotalFragments)
                                        {
                                            // signal upper layer of write request
                                            writeDoneHandler.call(writeBlock);

                                            // Clear shared pointer. This will free the previous
                                            // writeBlock's memory if it is no longer in use
                                            writeBlock = SharedPointer<Block>();

                                            // update length offset
                                            receiveLengthOffset += processedLength + currentPayloadLength;

                                            // update fragment offset
                                            receiveFragmentOffset = absoluteFragmentIndex + 1;

                                            // find max fragments in new writeBlock
                                            // Note: this calculation is rounding down
                                            uint16_t maxFragments = MAX_BLOCK_TRANSFER_SIZE / maxBlockPayloadSize;

                                            // total remaining fragments
                                            uint16_t remainingFragments = receiveTotalFragments - (absoluteFragmentIndex + 1);

                                            // check if the remaining fragments fit in a single batch transfer
                                            uint16_t fragments = (maxFragments < remainingFragments)
                                                                ? maxFragments : remainingFragments;

                                            // allocate memory
                                            uint8_t* buffer = (uint8_t*) malloc(fragments * maxBlockPayloadSize);
                                            uint32_t bufferLength = fragments * maxBlockPayloadSize;

                                            // only continue if allocation was successful
                                            if (buffer)
                                            {
                                                // allocate reference counted dynamic memory buffer
                                                writeBlock = SharedPointer<Block>(new BlockDynamic(buffer, bufferLength));

                                                // use IndexSet to keep track of the received fragments and find those missing
                                                missingFragments.setSize(fragments);

                                                BLE_DEBUG("bts: write next batch: %d %d\r\n", receiveFragmentOffset, receiveTotalFragments);

                                                // request fragments from next batch
                                                sendWriteRequestMissing();
                                            }
                                            else
                                            {
                                                // Error, received NULL pointer, abort transmission by sending acknowledgement
                                                sendWriteAcknowledgement();
                                            }
                                        }
                                        else
                                        {
                                            // received all, acknowledge the reception of the last fragment
                                            sendWriteAcknowledgement();
                                        }
                                    }
                                    else
                                    {
                                        // request missing fragments
                                        sendWriteRequestMissing();
                                    }
                                }
                            }
                            else
                            {
                                BLE_DEBUG("bts: write duplicate %d\r\n", relativeFragmentIndex);
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

                            BLE_DEBUG("bts: read request %d %d %d\r\n", sendFromFragment, sendAmount, readTotalFragments);

                            // send requested fragments
                            sendReadReply();
                        }
                        else
                        {
                            readState = BT_STATE_READY;

                            // reset connection handle if write is also ready
                            if (writeState == BT_STATE_READY)
                            {
                                connectionHandle = BTS_INVALID_HANDLE;
                            }

                            // signal read done
                            if (readDoneHandler)
                            {
                                readDoneHandler.call();
                            }

                            BLE_DEBUG("bts: read complete\r\n");
                        }
                    }
                    break;

                default:
                    BLE_DEBUG("bts: error - unknown message type (%02X)\r\n", messageType);
                    break;
            } // end message type switch
        } // end connection check
    } // end event->handle check
    // event->handle is different than writeToHandle
    else
    {
        BLE_DEBUG("bts: written %04X %04X\r\n", event->handle, writeToHandle);
    }
}

/* Connection status callbacks. Sets and resets variables and state. */
void BlockTransferService::onConnection(const Gap::ConnectionCallbackParams_t* params)
{
    BLE_DEBUG("bts: connected\r\n");
    (void) params;

    // increment connection counter
    connectionCounter++;

    // first connection, reset state
    if (connectionCounter == 1)
    {
        connectionHandle = BTS_INVALID_HANDLE;
        readState = BT_STATE_READY;
        writeState = BT_STATE_READY;
    }
}

void BlockTransferService::onDisconnection(const Gap::DisconnectionCallbackParams_t* params)
{
    BLE_DEBUG("bts: disconnected\r\n");

    // decrement connection counter
    connectionCounter--;

    // reset state if connection was the one being served
    if (params->handle == connectionHandle)
    {
        connectionHandle = BTS_INVALID_HANDLE;
        readState = BT_STATE_READY;
        writeState = BT_STATE_READY;
    }

    // close state if no connections are available
    if (connectionCounter == 0)
    {
        readState = BT_STATE_OFF;
        writeState = BT_STATE_OFF;
    }
}

/*  This function is called when the BLE device is ready for more characteristic value updates
    and is shared by all characteristics.
*/
void BlockTransferService::onDataSent(unsigned)
{
    BLE_DEBUG("bts: hvx sent\r\n");

    if (readState == BT_STATE_SERVER_READ)
    {
        sendReadReply();
    }

    if (writeState == BT_STATE_SERVER_WRITE_ACK)
    {
        sendWriteAcknowledgement();
    }
}

/*****************************************************************************/
/* Functions for sending fragments                                           */
/*****************************************************************************/

/*  Send missing fragments in current batch. For each successful transmission
    update variables to point to the next fragment.
*/
void BlockTransferService::sendReadReply(void)
{
    BLE_DEBUG("bts: read: reply\r\n");

    // while there are still fragments in this batch
    while (readFragmentsInBatch != 0)
    {
        // send current fragment
        bool result = sendReadReplyRepeatedly();

        BLE_DEBUG("\t: notify : %d %d\r\n", readFragmentIndex, result);

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
    // NULL pointer check
    if (readBlock)
    {
        // data already sent
        uint32_t processedLength = readFragmentIndex * maxBlockPayloadSize;

        // find length. the last packet might be shorter.
        // readFragmentIndex is zero-indexed
        uint16_t payloadLength = (readFragmentIndex < (readTotalFragments - 1))
                                ? maxBlockPayloadSize
                                : readBlock->getLength() - processedLength;

        // set data type based on whether more data is pending in this batch
        uint8_t type = (readFragmentsInBatch == 1) ? BT_TYPE_READ_PAYLOAD_LAST : BT_TYPE_READ_PAYLOAD_MORE;

        // insert header
        uint8_t payload[BLOCK_HEADER_SIZE + payloadLength];
        payload[0] = (type << 4) | (readFragmentIndex & 0x0F);
        payload[1] = readFragmentIndex >> 4;
        payload[2] = readFragmentIndex >> 12;

        // insert payload
        readBlock->memcpy(&(payload[3]), processedLength, payloadLength);

        // try to send fragment
        ble_error_t didSendValue = ble.gattServer().write(connectionHandle, readFromHandle, payload, BLOCK_HEADER_SIZE + payloadLength);

BLE_DEBUG("-> %d\r\n", didSendValue);

        return (didSendValue == BLE_ERROR_NONE);
    }

    return false;
}

/*  Function for requesting write fragments or signal reception complete.
*/
void BlockTransferService::sendWriteRequestMissing(void)
{
    BLE_DEBUG("bts: write missing\r\n");

    // Check IndexSet if there are still missing fragments in the current block
    if (missingFragments.getCount() > 0)
    {
        // find missing ranges
        uint32_t relativeFragmentIndex = 0;
        uint32_t count = 0;

        missingFragments.findMissing(&relativeFragmentIndex, &count);

        // add the fragment offset to get the absolute fragment number
        uint32_t absoluteFragmentIndex = relativeFragmentIndex + receiveFragmentOffset;

        // construct write request
        uint8_t request[7];

        request[0] = BT_TYPE_WRITE_REQUEST << 4;
        request[1] = absoluteFragmentIndex;
        request[2] = absoluteFragmentIndex >> 8;
        request[3] = absoluteFragmentIndex >> 16;
        request[4] = count;
        request[5] = count >> 8;
        request[6] = count >> 16;

        ble_error_t didSendValue = ble.gattServer().write(connectionHandle, readFromHandle, request, 7);
        (void) didSendValue;

        // set timeout for missing fragments
        timeout.attach_us(this, &BlockTransferService::fragmentTimeout, FRAGMENT_TIMEOUT_US);

        BLE_DEBUG("bts: write send request: %d: %d %d %d\r\n", connectionHandle, didSendValue, absoluteFragmentIndex, count);
    }
}

/*  Acknowledge all write fragments have been received.
*/
void BlockTransferService::sendWriteAcknowledgement()
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

    ble_error_t didSendValue = ble.gattServer().write(connectionHandle, readFromHandle, request, 7);

    if (didSendValue == BLE_ERROR_NONE)
    {
        BLE_DEBUG("bts: write send ack: 0xFFFFFF 1\r\n");

        // all done, reset state
        writeState = BT_STATE_READY;

        // reset connectionHandle if read is also ready
        if (readState == BT_STATE_READY)
        {
            connectionHandle = BTS_INVALID_HANDLE;
        }

        // signal upper layer of write request
        writeDoneHandler.call(writeBlock);

        // Clear shared pointer. This will free the previous
        // writeBlock's memory if it is no longer in use
        writeBlock = SharedPointer<Block>();
    }
    else
    {
        // acknowledgement not sent, try again when BLE is ready
        writeState = BT_STATE_SERVER_WRITE_ACK;
    }
}

/*  Timeout function called when fragments are not received in time.
*/
void BlockTransferService::fragmentTimeout(void)
{
    BLE_DEBUG("bts: timeout\r\n");

    // request missing fragments
    sendWriteRequestMissing();
}


