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

#include "ble-blocktransfer/BlockTransferClient.h"

#if 0
// increase timeout
#undef FRAGMENT_TIMEOUT_US
#define FRAGMENT_TIMEOUT_US (1000*1000)
#define BLE_DEBUG(...) { printf(__VA_ARGS__); }
#else
#define BLE_DEBUG(...) /* nothing */
#endif

/*****************************************************************************/
/* C to C++                                                                  */
/*****************************************************************************/

BlockTransferClient* btcBridge;

void bridgeCharacteristicDiscoveryCallback(const DiscoveredCharacteristic* characteristicP)
{
    if (btcBridge)
    {
        btcBridge->characteristicDiscoveryCallback(characteristicP);
    }
}

void bridgeReadCallback(const GattReadCallbackParams* params)
{
    if (btcBridge)
    {
        btcBridge->readCallback(params);
    }
}

void bridgeHVXCallback(const GattHVXCallbackParams* params)
{
    if (btcBridge)
    {
        btcBridge->hvxCallback(params);
    }
}

/*****************************************************************************/
/* BlockTransferClient                                                       */
/*****************************************************************************/


BlockTransferClient::BlockTransferClient()
    :   ble(),
        currentMTU(BTS_MTU_SIZE_DEFAULT),
        maxBlockPayloadSize(BTS_MTU_SIZE_DEFAULT - BLOCK_HEADER_SIZE),
        maxDirectReadPayloadSize(BTS_MTU_SIZE_DEFAULT - DIRECT_READ_HEADER_SIZE),
        internalState(BT_STATE_OFF)
{}

void BlockTransferClient::init(void (*clientReady)(void),
          const UUID& uuid,
          Gap::Handle_t _peripheral)
{
    peripheral = _peripheral;

    writeDoneHandler.attach(clientReady);

    ble.init();

    // register callback functions
    // this should be onDataWritten in gattClient, but interface does not support memberfunctions
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

ble_error_t BlockTransferClient::internalRead(Block* block)
{
    if (internalState == BT_STATE_READY)
    {
        readBlock = block;

        BLE_DEBUG("btc: read: %d\r\n", readBlock->getLength());

        /*  Do a normal characteristic read at given offset.
            The server will either respond with a single direct message with the data
            or a setup message for fragment requests.
        */
        internalState = BT_STATE_CLIENT_READ_SETUP;

        readCharacteristic.read(readBlock->getOffset());

        return BLE_ERROR_NONE;
    }
    else
    {
        return BLE_STACK_BUSY;
    }
}

ble_error_t BlockTransferClient::internalWrite(Block* block)
{
    BLE_DEBUG("btc: write\r\n");

    if (internalState == BT_STATE_READY)
    {
        writeBlock = block;

        /*  If the block is small enough to fit a single message, use direct shortcut
            and bypass the setup process.
        */
        if (writeBlock->getLength() <= MAX_DIRECT_WRITE_PAYLOAD_SIZE)
        {
            uint8_t length = 3 + writeBlock->getLength();
            uint8_t writeBuffer[length];

            writeBuffer[0] = BT_TYPE_WRITE_DIRECT << 4;
            writeBuffer[1] = writeBlock->getOffset();
            writeBuffer[2] = writeBlock->getOffset() >> 8;

            writeBlock->memcpy(&(writeBuffer[3]), 0, writeBlock->getLength());

            internalState = BT_STATE_CLIENT_WRITE_DIRECT;

            writeCharacteristic.writeWoResponse(length, writeBuffer);
        }
        else
        {
            //  Find the total number of fragments needed to transmit the block
            //  based on the MTU size minus payload header.
            outgoingTotalFragments = (writeBlock->getLength() + (maxBlockPayloadSize - 1)) / maxBlockPayloadSize;

            BLE_DEBUG("btc: write setup: %d %d %d\r\n", writeBlock->getLength(), outgoingTotalFragments, maxBlockPayloadSize);

            /*  Send setup message.
                When the receiver is ready for data it will send a request for fragments.
            */
            uint8_t length = 10;
            uint8_t writeBuffer[length];

            writeBuffer[0] = BT_TYPE_WRITE_SETUP << 4;

            writeBuffer[1] = writeBlock->getLength();
            writeBuffer[2] = writeBlock->getLength() >> 8;
            writeBuffer[3] = writeBlock->getLength() >> 16;

            writeBuffer[4] = writeBlock->getOffset();
            writeBuffer[5] = writeBlock->getOffset() >> 8;
            writeBuffer[6] = writeBlock->getOffset() >> 16;

            writeBuffer[7] = outgoingTotalFragments;
            writeBuffer[8] = outgoingTotalFragments >> 8;
            writeBuffer[9] = outgoingTotalFragments >> 16;

            internalState = BT_STATE_CLIENT_WRITE_SETUP;
            writeCharacteristic.writeWoResponse(length, writeBuffer);
        }

        return BLE_ERROR_NONE;
    }
    else
    {
        return BLE_STACK_BUSY;
    }
}

bool BlockTransferClient::writeInProgess(void)
{
    return (internalState != BT_STATE_OFF);
}

bool BlockTransferClient::readInProgress(void)
{
    return (internalState != BT_STATE_OFF);
}

bool BlockTransferClient::isReady(void)
{
    return (internalState == BT_STATE_OFF);
}

/*****************************************************************************/
/* Fragment functions                                                        */
/*****************************************************************************/

/*  Send missing fragments in current batch. For each successful transmission
    update variables to point to the next fragment.
*/
void BlockTransferClient::internalSendWriteReply(void)
{
    BLE_DEBUG("btc: write reply\r\n");

    // while there are still fragments in this batch
    while (outgoingFragmentsInBatch != 0)
    {
        // send current fragment
        bool result = internalSendWriteReplyRepeatedly();

        BLE_DEBUG("\t: write without response: %d %d\r\n", outgoingFragmentIndex, result);

        // update to next fragment if successful
        if (result)
        {
            outgoingFragmentIndex++;
            outgoingFragmentsInBatch--;
        }
        else
        {
            break;
        }
    }
}

/* Send the current fragment in this batch.
*/
bool BlockTransferClient::internalSendWriteReplyRepeatedly(void)
{
    // data already sent
    uint32_t processedLength = outgoingFragmentIndex * maxBlockPayloadSize;

    // find length. the last packet might be shorter.
    // outgoingFragmentIndex is zero-indexed
    uint16_t payloadLength = (outgoingFragmentIndex < (outgoingTotalFragments - 1))
                            ? maxBlockPayloadSize
                            : writeBlock->getLength() - processedLength;

    // set data type based on whether more data is pending in this batch
    uint8_t type = (outgoingFragmentsInBatch == 1) ? BT_TYPE_WRITE_PAYLOAD_LAST : BT_TYPE_WRITE_PAYLOAD_MORE;

    // insert header
    uint8_t payload[BLOCK_HEADER_SIZE + payloadLength];
    payload[0] = (type << 4) | (outgoingFragmentIndex & 0x0F);
    payload[1] = outgoingFragmentIndex >> 4;
    payload[2] = outgoingFragmentIndex >> 12;

    // insert payload
    writeBlock->memcpy(&(payload[3]), processedLength, payloadLength);

    // try to send fragment
    ble_error_t didSendValue = writeCharacteristic.writeWoResponse(BLOCK_HEADER_SIZE + payloadLength, payload);

    return (didSendValue == BLE_ERROR_NONE);
}

void BlockTransferClient::internalSendReadRequest()
{
    // Check IndexSet if there are still missing fragments in the current block
    if (missingFragments.getCount() > 0)
    {
        // find missing ranges
        uint32_t fragmentIndex = 0;
        uint32_t count = 0;

        missingFragments.findMissing(&fragmentIndex, &count);

        // construct read request
        uint8_t writeBuffer[7];

        writeBuffer[0] = BT_TYPE_READ_REQUEST << 4;
        writeBuffer[1] = fragmentIndex;
        writeBuffer[2] = fragmentIndex >> 8;
        writeBuffer[3] = fragmentIndex >> 16;
        writeBuffer[4] = count;
        writeBuffer[5] = count >> 8;
        writeBuffer[6] = count >> 16;

        writeCharacteristic.writeWoResponse(7, writeBuffer);

        // set timeout for missing fragments
        timeout.attach_us(this, &BlockTransferClient::fragmentTimeout, FRAGMENT_TIMEOUT_US);

        BLE_DEBUG("btc: read request: %d %d\r\n", fragmentIndex, count);
    }
}

void BlockTransferClient::internalSendReadAcknowledgement()
{
    // send acknowledgment for last fragment
    uint8_t writeBuffer[7];

    writeBuffer[0] = (BT_TYPE_READ_REQUEST << 4);
    writeBuffer[1] = 0xFF;
    writeBuffer[2] = 0xFF;
    writeBuffer[3] = 0xFF;
    writeBuffer[4] = 0x01;
    writeBuffer[5] = 0x00;
    writeBuffer[6] = 0x00;

    ble_error_t didSendValue = writeCharacteristic.writeWoResponse(7, writeBuffer);

    if (didSendValue == BLE_ERROR_NONE)
    {
        BLE_DEBUG("btc: read send ack: 0xFFFFFF 1\r\n");

        // all done, reset state
        internalState = BT_STATE_OFF;

        // signal upper layer of write request
        readDoneHandler.call(readBlock);
    }
    else
    {
        // acknowledgement not sent, try again when BLE is ready
        internalState = BT_STATE_CLIENT_READ_ACK;
    }
}

/*  Timeout function called when fragments are not received in time.
*/
void BlockTransferClient::fragmentTimeout(void)
{
    BLE_DEBUG("btc: timeout\r\n");

    // request missing fragments
    internalSendReadRequest();
}

/*****************************************************************************/
/* Event handlers                                                            */
/*****************************************************************************/

void BlockTransferClient::characteristicDiscoveryCallback(const DiscoveredCharacteristic* characteristicP)
{
    BLE_DEBUG("btc: discovered characteristic\r\n");
    BLE_DEBUG("btc: uuid: %04X %02X %02X\r\n", characteristicP->getUUID().getShortUUID(),
                                               characteristicP->getValueHandle(),
                                               *((uint8_t*)&(characteristicP->getProperties())));

    if (characteristicP->getProperties().notify())
    {
        BLE_DEBUG("btc: char: read\r\n");

        // save local copy
        readCharacteristic = *characteristicP;
    }
    else if (characteristicP->getProperties().writeWoResp())
    {
        BLE_DEBUG("btc: char: write\r\n");

        // save local copy
        writeCharacteristic = *characteristicP;
    }

    if ((readCharacteristic.getValueHandle() != GattAttribute::INVALID_HANDLE) &&
        (writeCharacteristic.getValueHandle() != GattAttribute::INVALID_HANDLE))
    {
        /* Note: Yuckiness alert! The following needs to be encapsulated in a neat API.
         * It isn't clear whether we should provide a DiscoveredCharacteristic::enableNoticiation() or
         * DiscoveredCharacteristic::discoverDescriptors() followed by DiscoveredDescriptor::write(...). */
        uint16_t value = BLE_HVX_NOTIFICATION;
        ble.gattClient().write(GattClient::GATT_OP_WRITE_REQ,
                               peripheral,
                               readCharacteristic.getValueHandle() + 1, /* HACK Alert. We're assuming that CCCD descriptor immediately follows the value attribute. */
                               sizeof(uint16_t),                          /* HACK Alert! size should be made into a BLE_API constant. */
                               reinterpret_cast<const uint8_t *>(&value));

        internalState = BT_STATE_READY;
        writeDoneHandler.call();
    }
}

void BlockTransferClient::readCallback(const GattReadCallbackParams* params)
{
    BLE_DEBUG("btc: read callback\r\n");

    if (params->handle == readCharacteristic.getValueHandle())
    {
#if 0
        for (std::size_t idx = 0; idx < params->len; idx++)
        {
            BLE_DEBUG("%02X", params->data[idx]);
        }
        BLE_DEBUG("\r\n");
#endif

        const uint8_t* message = params->data;
        const bt_type_t messageType = (bt_type_t) (message[0] >> 4);

        switch(messageType)
        {
            case BT_TYPE_READ_DIRECT:
                {
                    /*  Direct read message received.
                        Send payload to upper layer.
                    */
                    internalState = BT_STATE_OFF;

                    /*  Guard against read block being NULL. */
                    if (readBlock)
                    {
                        // payload length
                        uint16_t currentPayloadLength = params->len - DIRECT_READ_HEADER_SIZE;

                        // consider the length to avoid buffer overrun
                        if (currentPayloadLength > readBlock->getMaxLength())
                        {
                            currentPayloadLength = readBlock->getMaxLength();
                        }

                        readBlock->setLength(currentPayloadLength);

                        // copy payload
                        readBlock->memcpy(0, &(params->data[DIRECT_READ_HEADER_SIZE]), currentPayloadLength);

                        /*  Signal upper layer of read request.
                        */
                        // note: the read done handler is a FunctionPointerWithContext and does NULL pointer checks internally
                        readDoneHandler.call(readBlock);
                    }
                }
                break;

            case BT_TYPE_READ_SETUP:
                {
                    // guard against retransmissions resetting the state
                    if (internalState == BT_STATE_CLIENT_READ_SETUP)
                    {
                        /*  Read setup message received.
                        */

                        /*  Guard against read block being NULL. */
                        uint32_t maxLength = 0;

                        if (readBlock)
                        {
                            maxLength = readBlock->getMaxLength();
                        }

                        BLE_DEBUG("btc: read maxLength: %d\r\n", maxLength);

                        if (maxLength < maxBlockPayloadSize)
                        {
                            /*  Buffer is not large enough to hold a single fragment.
                                Signal client that transfer is done. The client knows
                                something is wrong because no fragments have been sent.
                            */
                            missingFragments.setSize(0);
                        }
                        else
                        {
                            uint32_t setupLength;
                            uint32_t setupFragments;

                            // block length, number is LSB
                            setupLength = message[3];
                            setupLength = (setupLength << 8) | message[2];
                            setupLength = (setupLength << 8) | message[1];

                            // fragments in block, number is send LSB
                            setupFragments = message[6];
                            setupFragments = (setupFragments << 8) | message[5];
                            setupFragments = (setupFragments << 8) | message[4];

                            // find read length that fits the receive buffer
                            if (setupLength > readBlock->getMaxLength())
                            {
                                setupLength = readBlock->getMaxLength();
                            }

                            incomingTotalLength = setupLength;

                            // find how many fragments we need to request based
                            // on the actual read length and payload size
                            incomingTotalFragments = (incomingTotalLength + (maxBlockPayloadSize - 1)) / maxBlockPayloadSize;

                            // set index set size when readblock is first called
                            missingFragments.setSize(incomingTotalFragments);

                            BLE_DEBUG("btc: read setup: %d %d %d\n\r", incomingTotalLength, incomingTotalFragments, maxBlockPayloadSize);
                        }

                        /*  Acknowledge setup by requesting data.
                        */
                        internalState = BT_STATE_CLIENT_READ_REQUEST;
                        internalSendReadRequest();
                    }
                }
                break;

            default:
                BLE_DEBUG("btc: unknown message type %02X\r\n", messageType);
                break;
        }
    }
}

void BlockTransferClient::hvxCallback(const GattHVXCallbackParams* params)
{
    if (params->handle == readCharacteristic.getValueHandle())
    {
        BLE_DEBUG("btc: hvx from read characteristic\r\n");

        const uint8_t* message = params->data;
        const bt_type_t messageType = (bt_type_t) (message[0] >> 4);

        switch(messageType)
        {
            case BT_TYPE_WRITE_REQUEST:
                {
                    // received request for sending a block of data
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
                    if (sendFromFragment < outgoingTotalFragments)
                    {
                        outgoingFragmentIndex = sendFromFragment;
                        outgoingFragmentsInBatch = sendAmount;

                        BLE_DEBUG("btc: write request %d %d %d\r\n", outgoingFragmentIndex, outgoingFragmentsInBatch, outgoingTotalFragments);

                        // send requested fragments
                        internalSendWriteReply();
                    }
                    else
                    {
                        internalState = BT_STATE_READY;
                        if (writeDoneHandler)
                        {
                            writeDoneHandler.call();
                        }
                        BLE_DEBUG("btc: write complete\r\n");
                    }
                }
                break;

            case BT_TYPE_READ_NOTIFY:
                {
                    BLE_DEBUG("btc: read notify\r\n");
                    if (notificationHandler)
                    {
                        notificationHandler.call();
                    }
                }
                break;

            case BT_TYPE_READ_PAYLOAD_MORE:
            case BT_TYPE_READ_PAYLOAD_LAST:
                {
                    if (internalState == BT_STATE_CLIENT_READ_REQUEST)
                    {
                        // reset timeout for missing fragments
                        timeout.attach_us(this, &BlockTransferClient::fragmentTimeout, FRAGMENT_TIMEOUT_US);

                        /*  Read fragment received and it was expected
                        */

                        uint32_t fragmentIndex;
                        fragmentIndex = message[2];
                        fragmentIndex = (fragmentIndex << 8) | message[1];
                        fragmentIndex = (fragmentIndex << 4) | (message[0] & 0xF);

                        // check if fragment is a duplicate
                        if (missingFragments.containsIndex(fragmentIndex))
                        {
                            // mark fragment as received in the index
                            missingFragments.removeIndex(fragmentIndex);

                            // copy payload to receive buffer
                            uint32_t currentPayloadLength = params->len - BLOCK_HEADER_SIZE;
                            uint32_t processedLength = fragmentIndex * maxBlockPayloadSize;

                            readBlock->memcpy(processedLength, &(params->data[BLOCK_HEADER_SIZE]), currentPayloadLength);

                            BLE_DEBUG("\t: fragment : %5d : ", fragmentIndex);
#if 0
                            for (std::size_t idx = 0; idx < currentPayloadLength; idx++)
                            {
                                BLE_DEBUG("%02X", message[idx]);
                            }
                            BLE_DEBUG(" : ");
#endif
                            BLE_DEBUG("%4d\r\n", processedLength);

                            /*  When sender signals "no more data", request missing fragments
                                or signal reception complete.
                            */
                            if (messageType == BT_TYPE_READ_PAYLOAD_LAST)
                            {
                                // last fragment received, cancel timeout
                                timeout.detach();

                                if (missingFragments.getCount() == 0)
                                {
                                    // update block offset and length
                                    readBlock->setLength(processedLength + currentPayloadLength);

                                    // acknowledge the reception of the last fragment
                                    internalSendReadAcknowledgement();
                                }
                                else
                                {
                                    // request missing fragments
                                    internalSendReadRequest();
                                }
                            }
                        }
                        else
                        {
                            BLE_DEBUG("btc: read duplicate %d\r\n", fragmentIndex);
                        }
                    }
                }
                break;

            default:
                BLE_DEBUG("btc: unknown type %02X\r\n", messageType);
                break;
        }
    }
}

void BlockTransferClient::internalDataSent(unsigned)
{
    BLE_DEBUG("btc: data sent\r\n");

    switch(internalState)
    {
        case BT_STATE_CLIENT_WRITE_DIRECT:
            // direct write payload complete
            BLE_DEBUG("btc: direct payload sent\n\r");

            internalState = BT_STATE_READY;
            if (writeDoneHandler)
            {
                writeDoneHandler.call();
            }
            break;

        case BT_STATE_CLIENT_WRITE_SETUP:
            // bulk write setup complete
            BLE_DEBUG("btc: write setup sent\n\r");

            // wait for write requests
            internalState = BT_STATE_CLIENT_WRITE_PAYLOAD;
            break;

        case BT_STATE_CLIENT_WRITE_PAYLOAD:
            // bulk write payload complete
            BLE_DEBUG("btc: payload sent\n\r");

            // process next payload in write sequence
            internalSendWriteReply();
            break;

        case BT_STATE_CLIENT_READ_ACK:
            // bulk read transfer complete, send ack
            internalSendReadAcknowledgement();
            break;

        case BT_STATE_READY:
        case BT_STATE_OFF: // do nothing; same as default
        default:
            break;
    }
}

/* Connection disconnected. Reset variables and state. */
void BlockTransferClient::internalOnDisconnection()
{
    BLE_DEBUG("btc: disconnected\r\n");

    internalState = BT_STATE_OFF;
}

