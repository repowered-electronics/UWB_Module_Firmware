#include "deca_mac.h"

uint8 make_mac_header(uint8_t* buffer, uint16_t dest_addr, uint16_t dest_pan_id, uint8_t seq_num){
	uint16_t mac_frame_ctrl = 0;
//	uint16_t src_addr = dwt_getaddress16();
//	uint16_t src_pan_id = dwt_getpanid();
	mac_frame_ctrl |= 1;	// indicate a DATA frame type
	mac_frame_ctrl |= (2 << 10); 	// 16-bit destination address
	mac_frame_ctrl |= (2 << 14); 	// 16-bit source address

	uint8 i = 0;
	buffer[i++] = mac_frame_ctrl & 0xFF;
	buffer[i++] = (mac_frame_ctrl >> 8) & 0xFF;
	buffer[i++] = seq_num;
	buffer[i++] = dest_pan_id & 0xFF;
	buffer[i++] = (dest_pan_id >> 8) & 0xFF;
	buffer[i++] = dest_addr & 0xFF;
	buffer[i++] = (dest_addr >> 8) & 0xFF;
	buffer[i++] = self_pan_id & 0xFF;
	buffer[i++] = (self_pan_id >> 8) & 0xFF;
	buffer[i++] = self_address & 0xFF;
	buffer[i++] = (self_address >> 8) & 0xFF;

	return i;
}

uint8 get_seq_number(uint8_t* buffer){
	return buffer[SEQ_NUM_INDEX];
}


uint16_t get_src_addr(uint8_t* buffer){
	uint16_t retval = buffer[9];
	retval |= buffer[10] << 8;
	return retval;
}

uint16_t get_src_panid(uint8_t* buffer){
	uint16_t retval = buffer[7];
	retval |= buffer[8] << 8;
	return retval;
}
