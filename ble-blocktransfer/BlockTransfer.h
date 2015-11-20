#ifndef __BLOCKTRANSFER_H__
#define __BLOCKTRANSFER_H__

/* allocate space to keep track of missing fragments. */
#define MAX_BLOCK_TRANSFER_SIZE         (1024)

/* maximum interval between fragments before recovery. */
#define FRAGMENT_TIMEOUT_MS             (100)

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

/* size constants */
#define BTS_MTU_SIZE_DEFAULT        20
#define DIRECT_READ_HEADER_SIZE     1
#define DIRECT_WRITE_HEADER_SIZE    3
#define BLOCK_HEADER_SIZE           3

#define MAX_BLOCK_PAYLOAD_SIZE          (BTS_MTU_SIZE_DEFAULT - BLOCK_HEADER_SIZE)
#define MAX_DIRECT_READ_PAYLOAD_SIZE    (BTS_MTU_SIZE_DEFAULT - DIRECT_READ_HEADER_SIZE)
#define MAX_DIRECT_WRITE_PAYLOAD_SIZE   (BTS_MTU_SIZE_DEFAULT - DIRECT_WRITE_HEADER_SIZE)

#define MAX_NUMBER_OF_FRAGMENTS         ((MAX_BLOCK_TRANSFER_SIZE + (MAX_BLOCK_PAYLOAD_SIZE - 1)) / MAX_BLOCK_PAYLOAD_SIZE)
#define MAX_INDEX_SET_SIZE              ((MAX_NUMBER_OF_FRAGMENTS + 7) / 8)


#endif // __BLOCKTRANSFER_H__
