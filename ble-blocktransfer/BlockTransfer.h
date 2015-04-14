#ifndef __BLOCKTRANSFER_H__
#define __BLOCKTRANSFER_H__

#include <stdint.h>

#include "mbed.h"

#include "ble-blocktransfer/Block.h"

/* state machine */
typedef enum {
    BT_STATE_CLIENT_SUBSCRIBE,
    BT_STATE_CLIENT_WRITE_DIRECT,
    BT_STATE_CLIENT_WRITE_SETUP,
    BT_STATE_CLIENT_WRITE_PAYLOAD,
    BT_STATE_CLIENT_READ_SETUP,
    BT_STATE_CLIENT_READ_REQUEST,
    BT_STATE_CLIENT_READ_ACK,
    BT_STATE_SERVER_READ,
    BT_STATE_SERVER_WRITE,
    BT_STATE_OFF
} bt_state_t;

extern bt_state_t blockTransferState;

/* message types */
typedef enum
{
    BT_TYPE_WRITE_SETUP         = 0x00,
    BT_TYPE_WRITE_REQUEST       = 0x01,
    BT_TYPE_WRITE_PAYLOAD_MORE  = 0x02,
    BT_TYPE_WRITE_PAYLOAD_LAST  = 0x03,
    BT_TYPE_WRITE_DIRECT        = 0x04,
    BT_TYPE_READ_SETUP          = 0x05,
    BT_TYPE_READ_REQUEST        = 0x06,
    BT_TYPE_READ_PAYLOAD_MORE   = 0x07,
    BT_TYPE_READ_PAYLOAD_LAST   = 0x08,
    BT_TYPE_READ_DIRECT         = 0x09,
    BT_TYPE_READ_NOTIFY         = 0x0A
} bt_type_t;

typedef enum
{
    BT_SUCCESS = 0,
    BT_ERROR = 1
} bt_error_t;


typedef void (* block_client_handler_t)(block_t* block, bt_error_t error);
typedef bt_error_t (* block_read_handler_t)(block_t* block);
typedef block_t* (* block_write_handler_t)(block_t* block);


/* size constants */
#define MTU_SIZE_DEFAULT            20 // GATT_MTU_SIZE_DEFAULT;
#define DIRECT_READ_HEADER_SIZE     1
#define DIRECT_WRITE_HEADER_SIZE    3
#define BLOCK_HEADER_SIZE           3

#define MAX_BLOCK_PAYLOAD_SIZE  (MTU_SIZE_DEFAULT - BLOCK_HEADER_SIZE)
#define MAX_DIRECT_READ_PAYLOAD_SIZE (MTU_SIZE_DEFAULT - DIRECT_READ_HEADER_SIZE)
#define MAX_DIRECT_WRITE_PAYLOAD_SIZE (MTU_SIZE_DEFAULT - DIRECT_WRITE_HEADER_SIZE)



#endif // __BLOCKTRANSFER_H__


