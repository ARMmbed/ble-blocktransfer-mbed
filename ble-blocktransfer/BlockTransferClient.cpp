#include "BlockTransferClient.h"

#include "BlockTransfer.h"

#include "IndexSet.h"
#include "mbed.h"

#include "app_timer.h"

#if 0
#define DEBUG(...) { printf(__VA_ARGS__); }
#else
#define DEBUG(...) /* nothing */
#endif


uint64_t timercounts;

//DigitalOut mbed_led1(P0_18);
DigitalOut mbed_led2(P0_19);
Ticker tick;
Timer watch;



/* variables for storing the current block being transmitted and the progress */
static block_t* currentWriteBlock = NULL;
static block_client_handler_t currentWriteHandler = NULL;
static uint16_t writeFragmentIndex = 0;
static uint16_t writeTotalFragments = 0;
static uint16_t writeFragmentsInBatch = 0;

/* variables for storing the current block being transmitted and the progress */
static block_t* currentReadBlock = NULL;
static block_client_handler_t currentReadHandler = NULL;
static uint16_t readNumberOfFragments;
static uint8_t readPayloadSize;                


/* handles for the current connection and characteristics */
static uint16_t connection_handle = 0;
static uint16_t read_handle = 0;
static uint16_t write_handle = 0;
static uint16_t cccd_notification_handle = 0; // unreliable

/* default data structure for interacting with the Softdevice */
static uint8_t buffer[MTU_SIZE_DEFAULT]; 

static ble_gattc_write_params_t write_params = {
    .write_op = 0,
    .handle = 0,
    .offset = 0,
    .len = MTU_SIZE_DEFAULT,
    .p_value = buffer,
    .flags = 0};




static uint8_t indexBuffer[30] = {0};
static index_t missingFragments;

static void blockWriteSequence();
static uint32_t blockWriteSequenceRepeat();

static void gattcReadEvent(ble_evt_t *p_ble_evt);
static void gattcWriteEvent(ble_evt_t *p_ble_evt);
static void gattcHVXEvent(ble_evt_t *p_ble_evt);
static void commonTxEvent(ble_evt_t *p_ble_evt);

static void nop() {;}
static uint32_t lastTime = 0;

/*  Given connection handle and database over server characteristics, find handles
    for characteristics used by the block transfer protocol and subscribe to notifications.
*/
bt_error_t blockClientAddService(uint16_t conn_handle, ble_db_discovery_srv_t* service)
{
    initIndexSet(&missingFragments, indexBuffer, 30);
    connection_handle = conn_handle;

    tick.attach(nop, 1.0);
    watch.start();

    // step through all characteristics in the given service
    for (uint8_t idx = 0; idx < service->char_count; idx++)
    {
        const ble_gattc_char_t* characteristic = &service->charateristics[idx].characteristic;
        const ble_gatt_char_props_t* properties = &characteristic->char_props;

        /* The Block Transfer protocol expects 4 characteristics:
            - read
            - read w/o response
            - write
            - write w/o response
        */
        if (properties->read == 1)
        {
            if (properties->notify == 1)
            {
                read_handle = characteristic->handle_value;
                cccd_notification_handle = service->charateristics[idx].cccd_handle;

                DEBUG("read: ");
            }
        }
        else if (properties->write_wo_resp == 1)
        {
            write_handle = characteristic->handle_value;
            DEBUG("write: ");
        }

        DEBUG("%04X %02X %04X\n\r", service->charateristics[idx].cccd_handle, 
                                                   characteristic->char_props, 
                                                   characteristic->handle_value);
    }

    /* if all four handles were found, do stuff */
    if ((read_handle != 0) && (write_handle != 0))
    {
        if (cccd_notification_handle != 0)
        {
            write_params.write_op = BLE_GATT_OP_WRITE_REQ; // write with response
            write_params.handle = cccd_notification_handle;
            write_params.len = 1;
            write_params.p_value[0] = BLE_GATT_HVX_NOTIFICATION; // notification with response

            blockTransferState = BT_STATE_CLIENT_SUBSCRIBE;
            sd_ble_gattc_write(connection_handle, &write_params);
        }

        return BT_SUCCESS;
    }
    else
    {
        return BT_ERROR;
    }
}


bt_error_t blockRead(block_t* block, block_client_handler_t handler)
{
    if (blockTransferState == BT_STATE_OFF)
    {
        currentReadBlock = block;
        currentReadHandler = handler;

        DEBUG("read: %d\n\r", currentReadBlock->length);

        lastTime = 0;

        /*  Do a normal characteristic read at given offset. 
            The server will either respond with a single direct message with the data
            or a setup message for fragment requests. 
        */
        blockTransferState = BT_STATE_CLIENT_READ_SETUP;
        sd_ble_gattc_read(connection_handle, read_handle, currentReadBlock->offset);

        return BT_SUCCESS;        
    }
    else
    {
        return BT_ERROR;
    }
}


void blockReadRequestMissing()
{
    uint16_t fragmentNumber; 
    uint16_t count;

    findMissing(&missingFragments, &fragmentNumber, &count);

    DEBUG("missing: %d %d\n\r", fragmentNumber, count);

    // found index
    if (count > 0)
    {
        /*  Send setup message.
            When the receiver is ready for data it will send a request for fragments.
        */
        write_params.write_op = BLE_GATT_OP_WRITE_CMD; // write without response
        write_params.handle = write_handle;
        write_params.len = 5; 

        write_params.p_value[0] = BT_TYPE_READ_REQUEST;
        write_params.p_value[1] = fragmentNumber; // LSB
        write_params.p_value[2] = fragmentNumber >> 8;
        write_params.p_value[3] = count;
        write_params.p_value[4] = count >> 8;

        sd_ble_gattc_write(connection_handle, &write_params);
     
        DEBUG("read request: %d %d\n\r", fragmentNumber, count);
    } 
    else 
    {
        /*  All data received. Acknowledge data reception by requesting fragment after block.
        */
        write_params.write_op = BLE_GATT_OP_WRITE_CMD; // write without response
        write_params.handle = write_handle;
        write_params.len = 5; 

        write_params.p_value[0] = BT_TYPE_READ_REQUEST;
        write_params.p_value[1] = 0xFF;
        write_params.p_value[2] = 0xFF;
        write_params.p_value[3] = 0x01;
        write_params.p_value[4] = 0x00;

        sd_ble_gattc_write(connection_handle, &write_params);
     
        DEBUG("read request: %d %d\n\r", readNumberOfFragments, count);

        blockTransferState = BT_STATE_OFF;
        currentReadHandler(currentReadBlock, BT_SUCCESS);    
    }
}


/*  Public function for transmitting a block of data and the function to be called afterwards.

    block_t block->data: data pointer
            block->length: data length 
            block->offset: The offset of this data block in relation to the overall characteristic.
                           Useful for writing large blocks that doesn't fit in RAM all at once but 
                           must be offloaded to/read from FLASH for example.
*/
bt_error_t blockWrite(block_t* block, block_client_handler_t handler)
{
    if (blockTransferState == BT_STATE_OFF)
    {
        lastTime = 0;

        currentWriteBlock = block;
        currentWriteHandler = handler;

        DEBUG("write: %d %s\n\r", currentWriteBlock->length, currentWriteBlock->data);

        /*  If the block is small enough to fit a single message, use direct shortcut
            and bypass the setup process.
        */
        if (currentWriteBlock->length <= MAX_DIRECT_WRITE_PAYLOAD_SIZE)
        {
            write_params.write_op = BLE_GATT_OP_WRITE_CMD; // write with response
            write_params.handle = write_handle;
            write_params.len = DIRECT_WRITE_HEADER_SIZE + currentWriteBlock->length;

            write_params.p_value[0] = BT_TYPE_WRITE_DIRECT; 
            write_params.p_value[1] = currentWriteBlock->offset;
            write_params.p_value[2] = currentWriteBlock->offset >> 8;
            memcpy(&(write_params.p_value[3]), currentWriteBlock->data, currentWriteBlock->length);

            blockTransferState = BT_STATE_CLIENT_WRITE_DIRECT;
            sd_ble_gattc_write(connection_handle, &write_params);
        }
        else
        {
            //  Find the total number of fragments needed to transmit the block
            //  based on the MTU size minus payload header.
            writeTotalFragments = currentWriteBlock->length / MAX_BLOCK_PAYLOAD_SIZE;

            if (writeTotalFragments * MAX_BLOCK_PAYLOAD_SIZE < currentWriteBlock->length)
            {
                writeTotalFragments++;
            }

            /*  Send setup message.
                When the receiver is ready for data it will send a request for fragments.
            */
            write_params.write_op = BLE_GATT_OP_WRITE_CMD; // write without response
            write_params.handle = write_handle;
            write_params.len = 7;

            write_params.p_value[0] = BT_TYPE_WRITE_SETUP;
            write_params.p_value[1] = currentWriteBlock->length; // LSB
            write_params.p_value[2] = currentWriteBlock->length >> 8;
            write_params.p_value[3] = currentWriteBlock->offset;
            write_params.p_value[4] = currentWriteBlock->offset >> 8;
            write_params.p_value[5] = writeTotalFragments; // LSB
            write_params.p_value[6] = writeTotalFragments >> 8;


            blockTransferState = BT_STATE_CLIENT_WRITE_SETUP;
            sd_ble_gattc_write(connection_handle, &write_params);
        }

        return BT_SUCCESS;
    }
    else
    {
        return BT_ERROR;
    }
}

/*  Wrapper function for calling blockWriteSequenceRepeat() repeatedly until all buffers are full. 
    Keeps track of progress variables.
*/
static void blockWriteSequence()
{
    uint32_t result = NRF_SUCCESS;

    while (writeFragmentsInBatch != 0)
    {
        result = blockWriteSequenceRepeat();

        DEBUG("write: %d %02X\n\r", writeFragmentIndex, result);   

        if (result == NRF_SUCCESS)
        {
            writeFragmentIndex++;
            writeFragmentsInBatch--;
        }
        else
        {
            break;
        }
    }
}

/*  Transmit the fragment corresponding to the current progress variables.
*/
static uint32_t blockWriteSequenceRepeat()
{
    uint32_t result;
    uint8_t length;
    uint8_t type;
    uint16_t index;

    // find length. the last packet might be shorter.
    // writeFragmentIndex is zero-indexed
    length = (writeFragmentIndex < writeTotalFragments - 1) ? MAX_BLOCK_PAYLOAD_SIZE :
                currentWriteBlock->length - ((writeTotalFragments - 1) * MAX_BLOCK_PAYLOAD_SIZE);  

    // set data type based on whether more data is pending in this batch
    type = (writeFragmentsInBatch == 1) ? BT_TYPE_WRITE_PAYLOAD_LAST : BT_TYPE_WRITE_PAYLOAD_MORE;

    write_params.write_op = BLE_GATT_OP_WRITE_CMD; // write without response
    write_params.handle = write_handle;
    write_params.len = BLOCK_HEADER_SIZE + length;
    write_params.p_value[0] = type; 
    write_params.p_value[1] = writeFragmentIndex; // LSB
    write_params.p_value[2] = writeFragmentIndex >> 8; // LSB

    index = writeFragmentIndex * MAX_BLOCK_PAYLOAD_SIZE;
    memcpy(&(write_params.p_value[3]), &(currentWriteBlock->data[index]), length);

    result = sd_ble_gattc_write(connection_handle, &write_params);

    return result;
}




/*  Public event handler for the Softdevice's BLE events
*/
void blockClientEventHandler(ble_evt_t *p_ble_evt)
{
    switch(p_ble_evt->header.evt_id)
    {
        /* GATT client events */
        // GATTC discovery handled by SD Discovery Service
#if 0
        case BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP:
        case BLE_GATTC_EVT_REL_DISC_RSP:
        case BLE_GATTC_EVT_CHAR_DISC_RSP:
        case BLE_GATTC_EVT_DESC_DISC_RSP:
        case BLE_GATTC_EVT_CHAR_VAL_BY_UUID_READ_RSP:
        case BLE_GATTC_EVT_CHAR_VALS_READ_RSP:
        case BLE_GATTC_EVT_TIMEOUT:
            break;
#endif

        case BLE_GATTC_EVT_READ_RSP:
            gattcReadEvent(p_ble_evt);
            break;

        case BLE_GATTC_EVT_WRITE_RSP:
            gattcWriteEvent(p_ble_evt);
            break;

        case BLE_GATTC_EVT_HVX:
            gattcHVXEvent(p_ble_evt);
            break;

        /* Common events */
        case BLE_EVT_TX_COMPLETE:
            commonTxEvent(p_ble_evt);
            break;

        default:
            break;
    }
}

static void gattcWriteEvent(ble_evt_t *p_ble_evt)
{
    const ble_gattc_evt_write_rsp_t* event = &p_ble_evt->evt.gattc_evt.params.write_rsp;

    if (event->handle == cccd_notification_handle)
    {
        lastTime = 0;

        switch(blockTransferState)
        {
            case BT_STATE_CLIENT_SUBSCRIBE:
                // subscribe to server read characteristic
                DEBUG("Subscribe sent\n\r");
                blockTransferState = BT_STATE_OFF;
                break;
            default:    
                break;
        }
    }
    else
    {
        DEBUG("Not a client cccd handle: %04X\n\r", event->handle);
    }
}

/*  The common event does not contain a handle to indicate which characteristic the transmission originated from.
    
*/
static void commonTxEvent(ble_evt_t *p_ble_evt)
{
    lastTime = 0;

    switch(blockTransferState)
    {
        case BT_STATE_CLIENT_WRITE_DIRECT:
            // direct write payload complete
            DEBUG("Direct payload sent\n\r");

            blockTransferState = BT_STATE_OFF;
            currentWriteHandler(currentWriteBlock, BT_SUCCESS);    
            break;

        case BT_STATE_CLIENT_WRITE_SETUP:
            // bulk write setup complete
            DEBUG("Write setup sent\n\r");

            // wait for write requests
            blockTransferState = BT_STATE_CLIENT_WRITE_PAYLOAD;
            break;

        case BT_STATE_CLIENT_WRITE_PAYLOAD:
            // bulk write payload complete
            DEBUG("Payload sent\n\r");

            // process next payload in write sequence
            blockWriteSequence();
            break;

        case BT_STATE_CLIENT_READ_ACK:
            // bulk read transfer ack complete
            blockTransferState = BT_STATE_OFF;
            break;

        case BT_STATE_OFF: // do nothing; same as default
        default:
            break;
    }

}



static void gattcReadEvent(ble_evt_t *p_ble_evt)
{
    const ble_gattc_evt_read_rsp_t* event = &p_ble_evt->evt.gattc_evt.params.read_rsp;


    /*  Only handle read response if it belongs to the client's read characteristic,
        since there might be other services running concurrently.
    */
    if (event->handle == read_handle)
    {
        lastTime = 0;

        switch(blockTransferState)
        {
            case BT_STATE_CLIENT_READ_SETUP:
                // received direct read data
                if (event->data[0] == BT_TYPE_READ_DIRECT)
                {
                    // consider the length to avoid buffer overrun
                    uint16_t length = (event->len > currentReadBlock->length) ? currentReadBlock->length : event->len;

                    // copy message to block buffer
                    memcpy(currentReadBlock->data, &(event->data[1]), length);

                    // set the length to the actual size of the data and not the size of the buffer
                    currentReadBlock->length = length;

                    blockTransferState = BT_STATE_OFF;
                    currentReadHandler(currentReadBlock, BT_SUCCESS);
                }
                else if (event->data[0] == BT_TYPE_READ_SETUP)
                {
                    // consider the length to avoid buffer overrun
                    uint16_t availableLength;
                    uint16_t availableFragments;
                    uint16_t currentLength;

                    // read characteristic size on server
                    availableLength = event->data[2];
                    availableLength = (availableLength << 8) | event->data[1];

                    // fragments on server
                    availableFragments = event->data[4];
                    availableFragments = (availableFragments << 8) | event->data[3];

                    // find payload size
                    readPayloadSize = availableLength / availableFragments;
                    if (readPayloadSize * availableFragments < availableLength)
                    {
                        readPayloadSize++;
                    }

                    // find read length that fits the receive buffer
                    currentLength = (availableLength > currentReadBlock->length) ? 
                                                currentReadBlock->length : availableLength;
                    currentReadBlock->length = currentLength;

                    // find how many fragments we need to request based
                    // on the actual read length and payload size
                    readNumberOfFragments = currentLength / readPayloadSize;

                    if (readNumberOfFragments * readPayloadSize < currentLength)
                    {
                        readNumberOfFragments++;
                    }

                    // check for index set size when readblock is first called
                    setIndexSetSize(&missingFragments, readNumberOfFragments);

                    DEBUG("Read request: %d %d %d\n\r", currentLength, readNumberOfFragments, readPayloadSize);

                    blockTransferState = BT_STATE_CLIENT_READ_REQUEST;
                    blockReadRequestMissing();
                }
                else
                {
                    DEBUG("Unexpected event message\n\r");
                }
                break;
            default:
                DEBUG("Unexpected state %02X\n\r", blockTransferState);
        }
    }
    else
    {
        DEBUG("Not a client read handle: %04X\n\r", event->handle);
    }
}

static void gattcHVXEvent(ble_evt_t *p_ble_evt)
{
    const ble_gattc_evt_hvx_t* event = &p_ble_evt->evt.gattc_evt.params.hvx;

    if (event->handle == read_handle)
    {
        lastTime = 0;

        switch(blockTransferState)
        {
            case BT_STATE_CLIENT_WRITE_PAYLOAD:
                // received request for sending a block of data
                if (event->data[0] == BT_TYPE_WRITE_REQUEST)
                {
                    uint16_t sendFromFragment;
                    uint16_t sendAmount;

                    // fragment index to send from
                    sendFromFragment = event->data[2];
                    sendFromFragment = (sendFromFragment << 8) | event->data[1];

                    // number of fragments requested
                    sendAmount = event->data[4];
                    sendAmount = (sendAmount << 8) | event->data[3];

                    // send fragments if request is within the valid range
                    if (sendFromFragment < writeTotalFragments)
                    {
                        DEBUG("Send %d\n\r", sendFromFragment);

                        // arm blockWriteSequence()
                        writeFragmentIndex = sendFromFragment;
                        writeFragmentsInBatch = sendAmount;

                        blockWriteSequence();
                    }
                    else
                    {
                        // the request was out-of-range which means all messages
                        // has been received
                        blockTransferState = BT_STATE_OFF;
                        currentWriteHandler(currentWriteBlock, BT_SUCCESS);    
                    }

                }
                else
                {
                    DEBUG("Unexpected event message\n\r");
                }
                break;
            case BT_STATE_CLIENT_READ_REQUEST:
                if ((event->data[0] == BT_TYPE_READ_PAYLOAD_MORE))
                {
                    uint16_t fragmentNumber;
                    uint16_t blockIndex;

                    fragmentNumber = event->data[2];
                    fragmentNumber = (fragmentNumber << 8) | event->data[1];

                    blockIndex = fragmentNumber * MAX_BLOCK_PAYLOAD_SIZE;

                    memcpy(&(currentReadBlock->data[blockIndex]), &(event->data[3]), event->len - 3),
                    removeIndex(&missingFragments, fragmentNumber);

                    DEBUG("%d\n\r", fragmentNumber);
                }
                else if (event->data[0] == BT_TYPE_READ_PAYLOAD_LAST)
                {
                    uint16_t fragmentNumber;
                    uint16_t blockIndex;

                    fragmentNumber = event->data[2];
                    fragmentNumber = (fragmentNumber << 8) | event->data[1];

                    blockIndex = fragmentNumber * MAX_BLOCK_PAYLOAD_SIZE;

                    memcpy(&(currentReadBlock->data[blockIndex]), &(event->data[3]), event->len - 3),
                    removeIndex(&missingFragments, fragmentNumber);

                    DEBUG("%d\n\r", fragmentNumber);

                    blockReadRequestMissing();
                }



                break;
            default:
                DEBUG("Unexpected state %02X\n\r", blockTransferState);
        }
    }
    else
    {
        DEBUG("Not a client read handle: %04X\n\r", event->handle);
    }
}

#if 0

void blockMainLoop()
{
    mbed_led2 = !mbed_led2;

    if (lastTime > 2)
    {
        switch(blockTransferState)
        {
            case BT_STATE_CLIENT_SUBSCRIBE:
                break;
            case BT_STATE_CLIENT_WRITE_DIRECT:
                // retransmit direct message
                sd_ble_gattc_write(connection_handle, &write_params);
                break;
            case BT_STATE_CLIENT_WRITE_SETUP:
            case BT_STATE_CLIENT_WRITE_PAYLOAD:
                // retransmit setup message
                {
                    uint32_t error = sd_ble_gattc_write(connection_handle, &write_params);
                    DEBUG("write setup: %d\n\r", error);
                }
                break;
#if 0
                // reconstruct the last payload
                writeFragmentIndex--;
                writeFragmentsInBatch = 1;

                blockWriteSequence();
                break;
#endif
            case BT_STATE_CLIENT_READ_SETUP:
                // no response to read request, retransmit
                {
                    uint32_t error = sd_ble_gattc_read(connection_handle, read_handle, currentReadBlock->offset);
                    DEBUG("read setup: %d\n\r", error);
                }
                break;
            case BT_STATE_CLIENT_READ_REQUEST:
                DEBUG("resend read request");
                blockReadRequestMissing();
                break;
            case BT_STATE_OFF:
            default:
                break;
        }

        DEBUG("tick: %d\n\r", watch.read_ms());
        lastTime = 0;
    }

}

#ifdef __cplusplus
extern "C" {
#endif
void TIMER1_IRQHandler()
{
    NRF_TIMER1->EVENTS_COMPARE[3] = 0;
    NRF_TIMER1->CC[3]             =  0x8000;
    NRF_TIMER1->TASKS_START = 1;

//    mbed_led1 = !mbed_led1;

    lastTime++;

    if (lastTime > 5)
    {
        NVIC_SetPendingIRQ(SWI0_IRQn);
    }
    else if (lastTime > 10)
    {
        NVIC_SetPendingIRQ(RTC1_IRQn);
        lastTime = 0;
    }

    app_timer_cnt_get(&timercounts);

}
#ifdef __cplusplus
}
#endif
#endif

