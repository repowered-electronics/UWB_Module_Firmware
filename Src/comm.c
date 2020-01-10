/*
 * comm.c
 *
 *  Created on: Aug 21, 2019
 *      Author: adam
 */
#include <comm.h>



int process_packet(uint8_t* pack_in, uint8_t* pack_out, CONFIG_FIELD_TYPE* fields, state_data_t* state){
	int ind = 0;
	int retval = PACKET_OK;
	switch(pack_in[0]){
	case CMD_READ_CONFIG:{
		pack_out[ind++] = pack_in[0];
		pack_out[ind++] = pack_in[1]*(FIELD_SIZE*sizeof(CONFIG_FIELD_TYPE) + 1); // set body length in bytes
		int i;
		for(i = HDDR_LEN; i < pack_in[1] + HDDR_LEN; i++){
			pack_out[ind++] = pack_in[i]; 					// copy field ID
			get_field(fields, pack_in[i], (void*)(pack_out + ind)); 	// copy field value
			ind += FIELD_SIZE;
		}
		pack_out[ind++] = STOP_BYTE;
		break;}
	case CMD_READ_ANCHORS:{
		pack_out[ind++] = pack_in[0];
		//get_field(fields, FIELD_NUMBER_OF_ANCHORS, (void*)num_anchors);
		int body_size = serialize_anchor_data(pack_out + HDDR_LEN, state->anchors, state->num_anchors);
		pack_out[ind++] = body_size; // length of the body
		ind += body_size;
		pack_out[ind] = STOP_BYTE;
		break;}
	case CMD_SET_CONFIG:{
		int i;
		for(i = HDDR_LEN; i < pack_in[1] + HDDR_LEN; i += FIELD_SIZE + 1){
			set_field(fields, pack_in[i], (void*)(pack_in + i + 1));
		}
		pack_out[0] = pack_in[0];
		pack_out[1] = 0;
		pack_out[2] = STOP_BYTE;
		break;}
	case CMD_RANGE:
		break;
	case CMD_RESTART:
		break;
	case CMD_RESET:
		break;
	case CMD_SAVE_CONFIG:
		//save_fields_to_eeprom(fields);
		pack_out[0] = pack_in[0];
		pack_out[1] = 0;
		pack_out[2] = STOP_BYTE;
		break;
	default:
		pack_out[0] = 0xFF;
		pack_out[1] = 0;
		pack_out[2] = STOP_BYTE;
//		retval = PACKET_CMD_ERROR;
		break;
	}
	return retval;
}


void read_config_from_eeprom(CONFIG_FIELD_TYPE* config){
//	HAL_FLASH_Unlock();
	uint32_t eeprom_data[NUM_FIELDS + 1];
	EE_Reads(0, NUM_FIELDS + 1, eeprom_data);
	if(eeprom_data[0] == EEPROM_FLAG){
		memcpy(config, eeprom_data + 1, NUM_FIELDS*FIELD_SIZE);
//		EE_Read(1+FIELD_SELF_ID, &eeprom_val);
//		memcpy(config + FIELD_SIZE*FIELD_SELF_ID, &eeprom_val, FIELD_SIZE);
//		EE_Read(1+FIELD_MODE, &eeprom_val);
//		memcpy(config + FIELD_SIZE*FIELD_MODE, &eeprom_val, FIELD_SIZE);
//		EE_Read(1+FIELD_CHANNEL, &eeprom_val);
//		memcpy(config + FIELD_SIZE*FIELD_CHANNEL, &eeprom_val, FIELD_SIZE);
//		EE_Read(1+FIELD_SAMPLES_PER_RANGE, &eeprom_val);
//		memcpy(config + FIELD_SIZE*FIELD_SAMPLES_PER_RANGE, &eeprom_val, FIELD_SIZE);
//		EE_Read(1+FIELD_NUMBER_OF_ANCHORS, &eeprom_val);
//		memcpy(config + FIELD_SIZE*FIELD_NUMBER_OF_ANCHORS, &eeprom_val, FIELD_SIZE);
//		EE_Read(1+FIELD_X, &eeprom_val);
//		memcpy(config + FIELD_SIZE*FIELD_X, &eeprom_val, FIELD_SIZE);
//		EE_Read(1+FIELD_Y, &eeprom_val);
//		memcpy(config + FIELD_SIZE*FIELD_Y, &eeprom_val, FIELD_SIZE);
//		EE_Read(1+FIELD_Z, &eeprom_val);
//		memcpy(config + FIELD_SIZE*FIELD_Z, &eeprom_val, FIELD_SIZE);
	}
//	HAL_FLASH_Lock();
}


void init_field_memory(CONFIG_FIELD_TYPE* fields){
	memset(fields, 0, NUM_FIELDS*FIELD_SIZE); // clear everything

	uint32_t temp = DFLT_SELF_ID;
	memcpy(fields + FIELD_SIZE*FIELD_SELF_ID, &temp, FIELD_SIZE);

	temp = DFLT_MODE;
	memcpy(fields + FIELD_SIZE*FIELD_MODE, &temp, FIELD_SIZE);

	temp = DFLT_CHANNEL;
	memcpy(fields + FIELD_SIZE*FIELD_CHANNEL, &temp, FIELD_SIZE);

	temp = DFLT_SAMPLES_PER_RANGE;
	memcpy(fields + FIELD_SIZE*FIELD_SAMPLES_PER_RANGE, &temp, FIELD_SIZE);

	temp = DFLT_NUMBER_OF_ANCHORS;
	memcpy(fields + FIELD_SIZE*FIELD_NUMBER_OF_ANCHORS, &temp, FIELD_SIZE);

}


void save_fields_to_eeprom(CONFIG_FIELD_TYPE* fields){
//	HAL_FLASH_Unlock();
	uint32_t eeprom_data[NUM_FIELDS + 1];
//	EE_Read(0, eeprom_data);
//	if(eeprom_data[0] != EEPROM_FLAG){
//
//	}
	EE_Format();
	eeprom_data[0] = EEPROM_FLAG;
	memcpy(eeprom_data + 1, fields, FIELD_SIZE*NUM_FIELDS);
	EE_Writes(0, NUM_FIELDS + 1, eeprom_data);
//	EE_Write(FIELD_SELF_ID + 1, eeprom_data);
//	memcpy(&eeprom_data, fields + FIELD_MODE*FIELD_SIZE, FIELD_SIZE);
//	EE_Write(FIELD_MODE + 1, eeprom_data);
//	memcpy(&eeprom_data, fields + FIELD_CHANNEL*FIELD_SIZE, FIELD_SIZE);
//	EE_Write(FIELD_CHANNEL + 1, eeprom_data);
//	memcpy(&eeprom_data, fields + FIELD_SAMPLES_PER_RANGE*FIELD_SIZE, FIELD_SIZE);
//	EE_Write(FIELD_SAMPLES_PER_RANGE + 1, eeprom_data);
//	memcpy(&eeprom_data, fields + FIELD_NUMBER_OF_ANCHORS*FIELD_SIZE, FIELD_SIZE);
//	EE_Write(FIELD_NUMBER_OF_ANCHORS + 1, eeprom_data);
//	memcpy(&eeprom_data, fields + FIELD_X*FIELD_SIZE, FIELD_SIZE);
//	EE_Write(FIELD_X + 1, eeprom_data);
//	memcpy(&eeprom_data, fields + FIELD_Y*FIELD_SIZE, FIELD_SIZE);
//	EE_Write(FIELD_Y + 1, eeprom_data);
//	memcpy(&eeprom_data, fields + FIELD_Z*FIELD_SIZE, FIELD_SIZE);
//	EE_Write(FIELD_Z + 1, eeprom_data);
//	HAL_FLASH_Lock();
}

void set_field(CONFIG_FIELD_TYPE* fields, int field_id, void* value){
	memcpy(fields + field_id * FIELD_SIZE, value, FIELD_SIZE);
}

void get_field(CONFIG_FIELD_TYPE* fields, int field_id, void* value){
	memcpy(value, fields + field_id*FIELD_SIZE, FIELD_SIZE);
}


int serialize_anchor_data(void* anchor_data, AnchorData* anchor_list, int list_size){
	int ind = 0;
	int size = 0;
	for(int i = 0; i < list_size; i++){
		if(anchor_list[i].distance <= 0.0 || anchor_list[i].timestamp == 0.0){
			continue;
		}
		memcpy(anchor_data + ind, &(anchor_list[i].id), size = sizeof(anchor_list[i].id));
		ind += size;
		memcpy(anchor_data + ind, &(anchor_list[i].timestamp), size = sizeof(anchor_list[i].timestamp));
		ind += size;
		memcpy(anchor_data + ind, &(anchor_list[i].x), size = sizeof(anchor_list[i].x));
		ind += size;
		memcpy(anchor_data + ind, &(anchor_list[i].y), size = sizeof(anchor_list[i].y));
		ind += size;
		memcpy(anchor_data + ind, &(anchor_list[i].z), size = sizeof(anchor_list[i].z));
		ind += size;
		memcpy(anchor_data + ind, &(anchor_list[i].distance), size = sizeof(anchor_list[i].distance));
		ind += size;
		memcpy(anchor_data + ind, &(anchor_list[i].rx_power), size = sizeof(anchor_list[i].rx_power));
		ind += size;
		memcpy(anchor_data + ind, &(anchor_list[i].fp_power), size = sizeof(anchor_list[i].fp_power));
		ind += size;
		memcpy(anchor_data + ind, &(anchor_list[i].fp_snr), size = sizeof(anchor_list[i].fp_snr));
		ind += size;
	}
	return ind;
}
