#ifndef _DECA_MAC_H_
#define _DECA_MAC_H_

#include <inttypes.h>
#include "deca_device_api.h"

#define MAC_SIZE_EXPECTED 	11
#define SEQ_NUM_INDEX 		2

#define SRC_ADDR_INDEX 		9

/**
 * @fn make_mac_header()
 *
 * @return - number of bytes written to the buffer
 */
uint16 self_pan_id;
uint16 self_address;
uint8 make_mac_header(uint8_t* buffer, uint16_t dest_addr, uint16_t dest_pan_id, uint8_t seq_num);
uint8 get_seq_number(uint8_t* buffer);
uint16_t get_src_addr(uint8_t* buffer);
uint16_t get_src_panid(uint8_t* buffer);

#endif
