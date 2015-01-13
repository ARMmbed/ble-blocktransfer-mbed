#ifndef __BLOCKTRANSFER_H__
#define __BLOCKTRANSFER_H__

#include <stdint.h>

#include "mbed.h"


#define NEED_CONSOLE_OUTPUT 1 /* Set this if you need debug messages on the console;
                               * it will have an impact on code-size and power consumption. */

#if NEED_CONSOLE_OUTPUT
extern Serial pc;
#define DEBUG(...) { pc.printf(__VA_ARGS__); }
#else
#define DEBUG(...) /* nothing */
#endif /* #if NEED_CONSOLE_OUTPUT */


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
    BT_TYPE_WRITE_SETUP,
    BT_TYPE_WRITE_REQUEST,
    BT_TYPE_WRITE_PAYLOAD_MORE,
    BT_TYPE_WRITE_PAYLOAD_LAST,
    BT_TYPE_WRITE_DIRECT,
    BT_TYPE_READ_SETUP,
    BT_TYPE_READ_REQUEST,
    BT_TYPE_READ_PAYLOAD_MORE,
    BT_TYPE_READ_PAYLOAD_LAST,
    BT_TYPE_READ_DIRECT,
    BT_TYPE_READ_NOTIFY
} bt_type_t;

typedef enum
{
    BT_SUCCESS = 0,
    BT_ERROR = 1
} bt_error_t;

typedef struct
{
    uint8_t* data;
    uint16_t length;
    uint16_t offset;
} block_t;

typedef void (* block_client_handler_t)(block_t* block, bt_error_t error);
typedef bt_error_t (* block_server_handler_t)(block_t* block);




/* size constants */
#define MTU_SIZE_DEFAULT            20 // GATT_MTU_SIZE_DEFAULT;
#define DIRECT_READ_HEADER_SIZE     1
#define DIRECT_WRITE_HEADER_SIZE    3
#define BLOCK_HEADER_SIZE           3

#define MAX_BLOCK_PAYLOAD_SIZE  (MTU_SIZE_DEFAULT - BLOCK_HEADER_SIZE)
#define MAX_DIRECT_READ_PAYLOAD_SIZE (MTU_SIZE_DEFAULT - DIRECT_READ_HEADER_SIZE)
#define MAX_DIRECT_WRITE_PAYLOAD_SIZE (MTU_SIZE_DEFAULT - DIRECT_WRITE_HEADER_SIZE)



#endif // __BLOCKTRANSFER_H__


