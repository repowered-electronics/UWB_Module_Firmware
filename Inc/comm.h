#ifndef _COMM_H_
#define _COMM_H_

#include <inttypes.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "rtls.h"
#include "eeprom.h"
#include "eepromConfig.h"
//#include "stm32f1xx_ll_usb.h"
//#include "usb_device.h"
//#include "usbd_cdc_if.h"
//#include "usbd_conf.h"
//#include "usbd_desc.h"

#define NUM_FIELDS 			8
#define CONFIG_FIELD_TYPE 	uint8_t
#define FIELD_SIZE 			4
#define FIELD_MEM_SIZE 		NUM_FIELDS*FIELD_SIZE
#define NUMBER_OF_ANCHORS 	5
#define ANCHOR_INFO_SIZE 	33
#define INPUT_TIMEOUT 		10
#define INPUT_BUFFER_SIZE 	512
#define HDDR_LEN 			2
#define STOP_BYTE 			0xA5
#define SEND_BUF_SIZE 		512


#define DFLT_SELF_ID 			0xDF
#define DFLT_MODE 	 			DEVICE_MODE_TAG
#define DFLT_CHANNEL 			(uint8_t)2
#define DFLT_SAMPLES_PER_RANGE 	3
#define DFLT_NUMBER_OF_ANCHORS 	5
#define DFLT_RANGING_PERIOD 	50


enum CMD_TYPES {
	CMD_READ_CONFIG		= 0x11,
	CMD_READ_ANCHORS 	= 0x12,
	CMD_SET_CONFIG 		= 0x22,
	CMD_RANGE 			= 0x33,
	CMD_RESTART 		= 0x44,
	CMD_RESET 			= 0x55,
	CMD_SAVE_CONFIG 	= 0x66
};

enum FIELDS {
	FIELD_SELF_ID 	= 0x00,
	FIELD_MODE 		= 0x01,
	FIELD_CHANNEL 	= 0x02,
	FIELD_SAMPLES_PER_RANGE 	= 0x03,
	FIELD_NUMBER_OF_ANCHORS 	= 0x04,
	FIELD_X = 0x05,
	FIELD_Y = 0x06,
	FIELD_Z = 0x07
};

enum PROCESS_PACKET_STATUS {
	PACKET_OK,
	PACKET_CMD_ERROR,
	PACKET_LENGTH_ERROR
};
AnchorData* anchor_data;

uint8_t usb_rx_buffer[INPUT_BUFFER_SIZE];

void init_field_memory(CONFIG_FIELD_TYPE* fields);
void read_config_from_eeprom(CONFIG_FIELD_TYPE* config);
void save_fields_to_eeprom(CONFIG_FIELD_TYPE* fields);
void set_field(CONFIG_FIELD_TYPE* fields, int field_id, void* value);
void get_field(CONFIG_FIELD_TYPE* fields, int field_id, void* value);
int serialize_device_config(void* self_data, void* config);
int serialize_anchor_data(void* anchor_data, AnchorData* anchor_list, int list_size);
int process_packet(uint8_t* pack_in, uint8_t* pack_out, CONFIG_FIELD_TYPE* fields, state_data_t* state);

#endif
