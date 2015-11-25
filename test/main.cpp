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

#include "mbed-drivers/mbed.h"
#include "ble/BLE.h"
#include "ble/DiscoveredCharacteristic.h"
#include "ble/DiscoveredService.h"

#include "ble-blocktransfer/BlockTransferClient.h"
#include "ble-blocktransfer/BlockTransferService.h"

#include "mbed-block/BlockStatic.h"
#include "mbed-block/BlockCollection.h"

/*****************************************************************************/
/* Configuration                                                             */
/*****************************************************************************/

#define VERBOSE_DEBUG_OUTPUT 1
#define GAP_CENTRAL_IS_GATT_CLIENT 1

const uint32_t maxWriteLength = 256;
const char DEVICE_NAME[] = "BlockTransfer";
const UUID uuid(0xAAAA);

/*****************************************************************************/
/* Variables                                                                 */
/*****************************************************************************/

// Transfer large blocks of data on platforms without Fragmentation-And-Recombination
static BlockTransferService bts;
static BlockTransferClient btc;

// debug led - blinks to show liveness
static Ticker ticker;
static DigitalOut mbed_led1(LED1);

// measure throughput
static Timer watch;

// buffers for sending and receiving data
static uint8_t writeBuffer[maxWriteLength];
SharedPointer<Block> writeBlock;

static uint8_t serverBuffer[maxWriteLength];
SharedPointer<Block> serverBlock;

// enable buttons to initiate transfer
static InterruptIn button1(BUTTON1);
static InterruptIn button2(BUTTON2);

// BLE book keeping
static bool scanning = false;
static Gap::Handle_t connectionHandle;
static bool peripheralIsServer = true;

// send notification when data is ready to be read
static void signalReady();
#if !defined(YOTTA_MINAR_VERSION_STRING)
static bool sendHello = false;
#endif

/*****************************************************************************/
/* Debug                                                                     */
/*****************************************************************************/
/*
    Called once every second. Blinks LED.
*/
void periodicCallback(void)
{
    mbed_led1 = !mbed_led1;
}

/*****************************************************************************/
/* Block Transfer Server                                                     */
/*****************************************************************************/

/*
    Function called when device receives a read request over BLE.
*/
SharedPointer<Block> blockServerReadHandler(uint32_t offset)
{
    printf("main: block read: %4lu %6lu\r\n", serverBlock->getLength(), offset);

#if VERBOSE_DEBUG_OUTPUT
    for (std::size_t idx = 0; idx < serverBlock->getLength(); idx++)
    {
        printf("%02X", serverBlock->at(idx));
    }
    printf("\r\n");
#endif

    return serverBlock;
}

/*
    Function called when read request is complete.
*/
void blockServerReadDoneHandler()
{
    printf("main: block read done\r\n");
}

/*
    Function called when data has been written over BLE.
*/
void blockServerWriteHandler(SharedPointer<BlockStatic> block)
{
    printf("main: block write: length: %4lu offset: %6lu time: %3d ms\r\n", block->getLength(), block->getOffset(), watch.read_ms());

    for (std::size_t idx = 0; idx < block->getLength(); idx++)
    {
        serverBlock->at(idx) = block->at(idx);

#if VERBOSE_DEBUG_OUTPUT
        printf("%02X", block->at(idx));
#endif
    }

#if VERBOSE_DEBUG_OUTPUT
    printf("\r\n");
#endif

    // the Block Transfer service is not busy
    if (bts.isReady())
    {
        // signal data is available to be read
        signalReady();
    }

    // reset timer to measure time to next block
    watch.reset();
}


/*****************************************************************************/
/* Block Transfer Client                                                     */
/*****************************************************************************/

/*
    Handler called when Block Transfer Client Write is finished.
*/
void clientWriteDone(SharedPointer<Block> block)
{
    printf("main: write done\r\n");
    (void) block;
}

/*
    Handler called when Block Transfer Client Read completes.
*/
void clientReadDone(SharedPointer<BlockStatic> block)
{
    printf("main: read done\r\n");

#if VERBOSE_DEBUG_OUTPUT
    // print block content
    for (std::size_t idx = 0; idx < block->getLength(); idx++)
    {
        printf("%02X", block->at(idx));
    }
    printf("\r\n");
#endif

    // compare read and write block
    bool matches = true;

    if (block->getLength() != writeBlock->getLength())
    {
        matches = false;
    }
    else
    {
        for (std::size_t idx = 0; idx < block->getLength(); idx++)
        {
            matches = matches && (block->at(idx) == writeBlock->at(idx));
        }
    }

    if (matches)
    {
        printf("main: buffers match\r\n");
    }
    else
    {
        printf("main: buffers differ\r\n");
    }

    // disconnect
    BLE::Instance().gap().disconnect(connectionHandle, Gap::REMOTE_USER_TERMINATED_CONNECTION);
}

/*
    Block Transfer Client received notification.

    Write to server is complete, and server signals that there is data to read.
*/
void clientNotification(SharedPointer<BlockStatic> block)
{
    printf("main: client notification\r\n");
    (void) block;

    btc.read(maxWriteLength, 0, clientReadDone);
}

/*
    Block Transfer Client has been initialized properbly when this handler is called.
*/
void clientReadyTask()
{
    // populate write buffer with data
    for (std::size_t idx = 0; idx < writeBlock->getLength(); idx++)
    {
        writeBlock->at(idx) = idx;
    }

#if VERBOSE_DEBUG_OUTPUT
    for (std::size_t idx = 0; idx < writeBlock->getLength(); idx++)
    {
        printf("%02X", writeBlock->at(idx));
    }
    printf("\r\n");
#endif

    // start transfer
    btc.write(writeBlock, clientWriteDone);
}

void clientReady()
{
    printf("main: client ready\r\n");

    minar::Scheduler::postCallback(clientReadyTask);
}

/*****************************************************************************/
/* BLE Central                                                               */
/*****************************************************************************/

bool advertisementContainsUUID(const Gap::AdvertisementCallbackParams_t* params, const UUID& uuid)
{
    // scan through advertisement data
    for (uint8_t idx = 0; idx < params->advertisingDataLen; )
    {
        uint8_t fieldLength = params->advertisingData[idx];
        uint8_t fieldType = params->advertisingData[idx + 1];
        const uint8_t* fieldValue = &(params->advertisingData[idx + 2]);

        // find 16-bit service IDs
        if ((fieldType == GapAdvertisingData::COMPLETE_LIST_16BIT_SERVICE_IDS) ||
            (fieldType == GapAdvertisingData::INCOMPLETE_LIST_16BIT_SERVICE_IDS))
        {
            uint8_t units = (fieldLength - 1) / 2;

            for (uint8_t idx = 0; idx < units; idx++)
            {
                // compare short UUID
                UUID beaconUUID((fieldValue[idx * 2 + 1] << 8) | fieldValue[idx * 2]);

                if (beaconUUID == uuid)
                {
                    return true;
                }
            }
        }
        // find 128-bit service IDs
        else if ((fieldType == GapAdvertisingData::COMPLETE_LIST_128BIT_SERVICE_IDS) ||
                 (fieldType == GapAdvertisingData::INCOMPLETE_LIST_128BIT_SERVICE_IDS))
        {
            uint8_t units = (fieldLength - 1) / 16;

            for (uint8_t idx = 0; idx < units; idx++)
            {
                // compare long UUID
                UUID beaconUUID(&fieldValue[idx * 16]);

                if (beaconUUID == uuid)
                {
                    return true;
                }
            }
        }

        // move to next field
        idx += fieldLength + 1;
    }

    return false;
}

/*
    Handler for when advertisement beacons are received.
*/
void advertisementCallback(const Gap::AdvertisementCallbackParams_t* params)
{
#if VERBOSE_DEBUG_OUTPUT
    printf("adv peerAddr[%02x %02x %02x %02x %02x %02x] rssi %d, isScanResponse %u, AdvertisementType %u\r\n",
           params->peerAddr[5], params->peerAddr[4], params->peerAddr[3], params->peerAddr[2], params->peerAddr[1], params->peerAddr[0],
           params->rssi, params->isScanResponse, params->type);
#endif

    // if advertisement contains the uuid we are looking for, connect to peripheral
    if (advertisementContainsUUID(params, uuid))
    {
        // set lowest latency for fastest transfer
        Gap::ConnectionParams_t fast;
        BLE::Instance().gap().getPreferredConnectionParams(&fast);
        fast.minConnectionInterval = 16; // 20 ms
        fast.maxConnectionInterval = 32; // 40 ms
        fast.slaveLatency = 0;

        // connect to peripheral, this stops the scanning
        BLE::Instance().gap().connect(params->peerAddr, Gap::ADDR_TYPE_RANDOM_STATIC, &fast, NULL);
    }
}

/*
    Functions called when BLE device has been connected.
*/
void whenConnected(const Gap::ConnectionCallbackParams_t* params)
{
    printf("main: Connected: %d %d %d\r\n", params->connectionParams->minConnectionInterval,
                                            params->connectionParams->maxConnectionInterval,
                                            params->connectionParams->slaveLatency);

    // save connection handle
    connectionHandle = params->handle;

    if (params->role == Gap::CENTRAL)
    {
        printf("main: central\r\n");
        scanning = false;

        // Instantiate a BlockTransferClient object if the peripheral is in server mode
        if (peripheralIsServer == true)
        {
            // initialize Block Transfer client
            btc.init(clientReady, uuid, params->handle);
            btc.onNotification(clientNotification);
        }
        else
        {
            watch.start();
        }
    }
    else
    {
        printf("main: peripheral\r\n");

        // Instantiate a BlockTransferClient object if the peripheral is not in server mode
        if (peripheralIsServer == false)
        {
            // initialize Block Transfer client
            btc.init(clientReady, uuid, params->handle);
            btc.onNotification(clientNotification);
        }
        else
        {
            watch.start();
        }
    }
}

/*
    Function called when BLE device has been disconnected.
*/
void whenDisconnected(const Gap::DisconnectionCallbackParams_t*)
{
    printf("main: Disconnected!\r\n");
    printf("main: Restarting the advertising process\r\n");

    watch.stop();
    watch.reset();

    BLE::Instance().gap().startAdvertising();
}

/*****************************************************************************/
/* Buttons                                                                   */
/*****************************************************************************/

/*
    Start or stop scanning. Device becomes central.
*/
void button1Task()
{
    if (!scanning)
    {
        scanning = true;
        printf("start scan\r\n");
        BLE::Instance().gap().setScanParams(1000, 1000);
        BLE::Instance().gap().startScan(advertisementCallback);
    }
    else
    {
        scanning = false;
        printf("stop scan\r\n");
        BLE::Instance().gap().stopScan();
    }
}

void button2Task()
{
    if (peripheralIsServer)
    {
        peripheralIsServer = false;
        printf("Peripheral is now GATT Client\r\n");
    }
    else
    {
        peripheralIsServer = true;
        printf("Peripheral is now GATT Server\r\n");
    }
}

void button1ISR()
{
    minar::Scheduler::postCallback(button1Task);
}

void button2ISR()
{
    minar::Scheduler::postCallback(button2Task);
}

/*****************************************************************************/
/* App start                                                                 */
/*****************************************************************************/

void initDone(BLE::InitializationCompleteCallbackContext* context)
{
    (void) context;

    BLE& ble = BLE::Instance();

    // connection status handlers
    ble.gap().onConnection(whenConnected);
    ble.gap().onDisconnection(whenDisconnected);

    /* construct advertising beacon */
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED|GapAdvertisingData::LE_GENERAL_DISCOVERABLE);
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::SHORTENED_LOCAL_NAME, (const uint8_t *) DEVICE_NAME, sizeof(DEVICE_NAME) - 1);

    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_16BIT_SERVICE_IDS, uuid.getBaseUUID(), uuid.getLen());

    ble.gap().setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
    ble.gap().setAdvertisingInterval(200);

    // Apple uses device name instead of beacon name
    ble.gap().setDeviceName((const uint8_t*) DEVICE_NAME);

    // ble setup complete - start advertising
    ble.gap().startAdvertising();

    /*************************************************************************/
    /*************************************************************************/
    bts.init(uuid, SecurityManager::SECURITY_MODE_ENCRYPTION_OPEN_LINK);

    // set callback functions
    bts.setWriteAuthorizationCallback(blockServerWriteHandler);
    bts.setReadAuthorizationCallback(blockServerReadHandler);
    bts.setReadDoneCallback(blockServerReadDoneHandler);

    serverBlock = SharedPointer<Block>(new BlockStatic(serverBuffer, sizeof(serverBuffer)));
    writeBlock = SharedPointer<Block>(new BlockStatic(writeBuffer, sizeof(writeBuffer)));

    {
        uint8_t* heap = (uint8_t*) malloc(1);
        printf("BlockTransfer: %s %s %d %p %p\r\n", __DATE__, __TIME__, OPEN_MAX, heap, &heap);
        free(heap);
    }
}

void app_start(int, char *[])
{
    // setup buttons
    button1.mode(PullUp);
    // Delay for initial pullup to take effect
    wait(.01);
    button1.fall(button1ISR);

    button2.mode(PullUp);
    // Delay for initial pullup to take effect
    wait(.01);
    button2.fall(button2ISR);

    // blink led
    ticker.attach(periodicCallback, 1.0);

    /*************************************************************************/
    /*************************************************************************/
    /* bluetooth le */
    BLE::Instance().init(initDone);
}

/*****************************************************************************/
/* Compatibility                                                             */
/*****************************************************************************/

#if defined(YOTTA_MINAR_VERSION_STRING)
/*********************************************************/
/* Build for mbed OS                                     */
/*********************************************************/
/*
    Task for sending notification.

    This signals the client to perform a read.
*/
void sendNotificationTask()
{
    ble_error_t result = bts.updateCharacteristicValue((const uint8_t*)"hello", 5);

    // retry
    if (result != BLE_ERROR_NONE)
    {
        minar::Scheduler::postCallback(sendNotificationTask);
    }
}

void signalReady()
{
    minar::Scheduler::postCallback(sendNotificationTask);
}

#else
/*********************************************************/
/* Build for mbed Classic                                */
/*********************************************************/

void signalReady()
{
    sendHello = true;
}

int main(void)
{
    app_start(0, NULL);

    for(;;)
    {
        // send notification outside of interrupt context
        if (sendHello)
        {
            ble_error_t result = bts.updateCharacteristicValue((const uint8_t*)"hello", 5);

            if (result == BLE_ERROR_NONE)
            {
                sendHello = false;
            }
        }

        BLE::Instance().waitForEvent();
    }
}
#endif
