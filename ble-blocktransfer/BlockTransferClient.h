#ifndef __BLOCKTRANSFERCLIENT_H__
#define __BLOCKTRANSFERCLIENT_H__

#ifdef __cplusplus
//extern "C" {
#endif

#define MTU_SIZE_DEFAULT            20 // GATT_MTU_SIZE_DEFAULT;
#define MAX_BLOCK_PAYLOAD_SIZE  (MTU_SIZE_DEFAULT - BLOCK_HEADER_SIZE)
#define MAX_DIRECT_READ_PAYLOAD_SIZE (MTU_SIZE_DEFAULT - DIRECT_READ_HEADER_SIZE)
#define MAX_DIRECT_WRITE_PAYLOAD_SIZE (MTU_SIZE_DEFAULT - DIRECT_WRITE_HEADER_SIZE)

#include <ble_db_discovery.h>

#include "ble-blocktransfer/BlockTransfer.h"

bt_error_t blockClientAddService(uint16_t conn_handle, ble_db_discovery_srv_t* database);
bt_error_t blockRead(block_t* block, block_client_handler_t handler);
bt_error_t blockWrite(block_t* block, block_client_handler_t handler);

void blockClientEventHandler(ble_evt_t *p_ble_evt);
//void blockMainLoop();

#ifdef __cplusplus
//}
#endif

#endif // __BLOCKTRANSFERCLIENT_H__