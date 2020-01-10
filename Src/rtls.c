/*
 * rtls.c
 *
 *  Created on: Dec 16, 2019
 *      Author: adam
 */


#include "rtls.h"


RangingStatus range_with_anchor(uint8_t anchor_id, AnchorTimeStamps* a_stamps, BeaconTimeStamps* b_stamps, uint8* seq_num){
	uint8 tx_buf[FRAME_LEN_MAX];
	uint8 rx_buf[FRAME_LEN_MAX];
	uint8 data_len;
	uint16 new_src_addr;
	uint16 new_src_panid;
	uint16 new_seq_num;
	TxStatus tx_status;
	int rcv_len = -1;

	// ----- SEND POLL -----
	data_len = make_mac_header(tx_buf, anchor_id, anchor_id, *seq_num);
	tx_buf[data_len++] = POLL;
	HAL_Delay(DELAY_BEFORE_TX);

	tx_status = transmit_frame(tx_buf, data_len + 2, 1); 	// data_len + 2 to account for checksum

	if(!(tx_status == TX_SUCCESS)){
		return SEND_POLL_FAILED;
	}
	b_stamps->t_sp = get_tx_timestamp();

	// ----- RECEIVE RESPONSE -----
	rcv_len = receive_frame(rx_buf, FRAME_LEN_MAX, 1000);
	if(rcv_len <= 0){
		// debug(&huart1, "No frame received\r");
		return RECEIVE_RESPONSE_FAILED;
	}
	new_src_addr 	= get_src_addr(rx_buf);
	new_src_panid 	= get_src_panid(rx_buf);
	new_seq_num 	= get_seq_number(rx_buf);
	if(new_src_addr != anchor_id || new_src_panid != anchor_id){
		// debug(&huart1, "Received from unexpected source\r");
		return RECEIVE_RESPONSE_FAILED;
	}
	if(rx_buf[MAC_SIZE_EXPECTED] != RESPONSE_INIT){
		// debug(&huart1, "Frame didn't contain the right numbers\r");
		return RECEIVE_RESPONSE_FAILED;
	}

	*seq_num = get_seq_number(rx_buf);
	b_stamps->t_rr = get_rx_timestamp();

	// ----- SEND FINAL -----
	data_len = make_mac_header(tx_buf, anchor_id, anchor_id, ++(*seq_num));
	tx_buf[data_len++] = SEND_FINAL;
	HAL_Delay(DELAY_BEFORE_TX);

	tx_status = transmit_frame(tx_buf, data_len + 2, 1);

	if(!(tx_status == TX_SUCCESS)){
		return SEND_FINAL_FAILED;
	}

	b_stamps->t_sf = get_tx_timestamp();

	// ----- RECEIVE TIMESTAMPS -----
	rcv_len = receive_frame(rx_buf, FRAME_LEN_MAX, 1000);
	if(rcv_len <= 0){
		// debug(&huart1, "No timestamps frame received\r");
		return RECEIVE_TIMESTAMPS_FAILED;
	}
	new_src_addr 	= get_src_addr(rx_buf);
	new_src_panid 	= get_src_panid(rx_buf);
	new_seq_num 	= get_seq_number(rx_buf);
	if(new_src_addr != anchor_id || new_src_panid != anchor_id){
		//debug(&huart1, "Received timestamps from unexpected source\r");
		return RECEIVE_TIMESTAMPS_FAILED;
	}
	*seq_num = get_seq_number(rx_buf);
	b_stamps->t_ff = get_rx_timestamp();
	if(rx_buf[MAC_SIZE_EXPECTED] != RESPONSE_DATA){
		//debug(&huart1, "Timestamps frame didn't contain the right numbers\r");
		return RECEIVE_TIMESTAMPS_FAILED;
	}
	memcpy(a_stamps, rx_buf+MAC_SIZE_EXPECTED+1, sizeof(*a_stamps));

	return RANGING_SUCCESS;
}

int receive_frame(uint8* buffer, int max_len, int timeout){
	/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
	uint32 status_reg = 0;

	/* Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint. */
	uint16 frame_len = 0;

	int count = 0;
	timeout = timeout * 20; // convert milliseconds to 50's of uSeconds

	dwt_rxenable(DWT_START_RX_IMMEDIATE); /* Activate reception immediately. See NOTE 3 below. */

	/* Poll until a frame is properly received or an error/timeout occurs. See NOTE 4 below.
	 * STATUS register is 5 bytes long but, as the event we are looking at is in the first byte of the register, we can use this simplest API
	 * function to access it. */
	while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)) && !(count >= timeout))
	{
		//		HAL_Delay(1);
		u_delay(50);
		count++;
	};

	if (status_reg & SYS_STATUS_RXFCG)
	{
		/* A frame has been received, copy it to our local buffer. */
		frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
		if (frame_len <= max_len)
		{
			dwt_readrxdata(buffer, frame_len, 0);
		}

		/* Clear good RX frame event in the DW1000 status register. */
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

		return frame_len;
	}
	else if(count >= timeout)
	{
		dwt_forcetrxoff(); // guarantee that the receiver is off
		return -1;
	}else{
		/* Clear RX error events in the DW1000 status register. */
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);

		/* Reset RX to properly reinitialise LDE operation. */
		dwt_rxreset();
		return -1;
	}
}
 /**
  * Called when we believe a frame has been received
  */
int read_rx_frame(uint8* buffer){
	/* Assuming a frame has been received, copy it to our local buffer. */
	uint16 frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
	if (frame_len <= FRAME_LEN_MAX){
		dwt_readrxdata(buffer, frame_len, 0);
	}

	/* Clear good RX frame event in the DW1000 status register. */
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

	return frame_len;
}

void send_response_init(state_data_t* state_data){
	state_data->transact_id = get_src_addr(state_data->rx_buffer);
	uint16_t src_address 	= get_src_addr(state_data->rx_buffer);
	uint16_t src_panid 		= get_src_panid(state_data->rx_buffer);
	uint8_t sequence_num 	= get_seq_number(state_data->rx_buffer);
	uint8 data_len 			= rtls_make_mac_header(state_data, FRAME_TYPE_DATA);//tx_buffer, src_address, src_panid, ++sequence_num);
	state_data->tx_buffer[data_len++] 	= RESPONSE_INIT;
	state_data->tx_status = transmit_frame(state_data->tx_buffer, data_len + 2, 1); // +2 to account for checksum

	state_data->t_sr = get_tx_timestamp();
}

void send_response_final(state_data_t* state){
	uint8 data_len 					= rtls_make_mac_header(state, FRAME_TYPE_DATA);
	state->tx_buffer[data_len++] 	= SEND_FINAL;
	transmit_frame(state->tx_buffer, data_len + 2, 1); // +2 to account for checksum

	state->t_sf = get_tx_timestamp();
}

void send_response_data(state_data_t* state){
	uint8 data_len = rtls_make_mac_header(state, FRAME_TYPE_DATA);
//	uint8 data_len = make_mac_header(state->tx_buffer, state->transact_id, state->transact_id, state->seq_num);
	state->tx_buffer[data_len++] = RESPONSE_DATA;

	memcpy(state->tx_buffer+data_len, &state->t_rp, sizeof(state->t_rp));
	data_len += sizeof(state->t_rp);

	memcpy(state->tx_buffer+data_len, &state->t_sr, sizeof(state->t_sr));
	data_len += sizeof(state->t_sr);

	memcpy(state->tx_buffer+data_len, &state->t_rf, sizeof(state->t_rf));
	data_len += sizeof(state->t_rf);

	memcpy(state->tx_buffer+data_len, &state->x, sizeof(state->x));
	data_len += sizeof(state->x);

	memcpy(state->tx_buffer+data_len, &state->y, sizeof(state->y));
	data_len += sizeof(state->y);

	memcpy(state->tx_buffer+data_len, &state->z, sizeof(state->z));
	data_len += sizeof(state->z);

	transmit_frame(state->tx_buffer, data_len + 2, 1);
	state->t_rp = 0;
	state->t_sr = 0;
	state->t_rf = 0;
}


void send_anchor_broadcast(state_data_t* state){
	//state->transact_id = 0xFFFF;
	int ind = rtls_make_mac_header(state, FRAME_TYPE_BEACON);
	state->tx_buffer[ind++] = ANCHOR_BROADCAST;
	memcpy(state->tx_buffer + ind, &state->x, sizeof(state->x));
	ind += sizeof(state->x);
	memcpy(state->tx_buffer + ind, &state->y, sizeof(state->y));
	ind += sizeof(state->y);
	memcpy(state->tx_buffer + ind, &state->z, sizeof(state->z));
	ind += sizeof(state->z);

	transmit_frame(state->tx_buffer, ind + 2, 0);
}


TxStatus transmit_frame(uint8* frame, int f_len, _Bool ranging){
	/* Write frame data to DW1000 and prepare transmission. See NOTE 4 below.*/
	dwt_writetxdata(f_len, frame, 0); 			/* Zero offset in TX buffer. */
	dwt_writetxfctrl(f_len, 0, ranging); 		/* Zero offset in TX buffer */
	dwt_starttx(DWT_START_TX_IMMEDIATE); 	// Start transmission

	/* Poll DW1000 until TX frame sent event set.
	 * STATUS register is 5 bytes long but, as the event we are looking at is in the first byte of the register, we can use this simplest API
	 * function to access it.*/
	int count = 0;
	while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS) && count < 100){
		count++;
		u_delay(100); // 0.1ms
	};
	if(count >= 100){
		return TX_TIMEOUT;
	}
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS); /* Clear TX frame sent event. */

	return TX_SUCCESS;
}

unsigned long long get_tx_timestamp(void){
	uint8 data[5];
	dwt_readtxtimestamp(data);
	unsigned long long retval = 0;

	for (int i = 4; i >= 0; i--)
	{
		retval <<= 8;
		retval |= data[i];
	}
	return retval;
}

unsigned long long get_rx_timestamp(void){
	uint8 data[5];
	dwt_readrxtimestamp(data);
	unsigned long long retval = 0;

	for (int i = 4; i >= 0; i--)
	{
		retval <<= 8;
		retval |= data[i];
	}
	return retval;
}

double get_fp_power(dwt_rxdiag_t* diagnostics){
	double F1 = 1.0 * diagnostics->firstPathAmp1;
	double F2 = 1.0 * diagnostics->firstPathAmp2;
	double F3 = 1.0 * diagnostics->firstPathAmp3;
	double A  = 121.74; // for PRF of 64 MHz (see pg. 45 of DW1000 user manual)
	double N  = 1.0 * diagnostics->rxPreamCount;

	double retval = ( pow(F1, 2.0) + pow(F2, 2.0) + pow(F3, 2.0) ) / pow(N, 2.0);
	return 10.0 * log10(retval) - A;
}
double get_rx_power(dwt_rxdiag_t* diagnostics){
	double C = 1.0 * diagnostics->maxGrowthCIR;
	double A  = 121.74; // for PRF of 64 MHz (see pg. 45 of DW1000 user manual)
	double N  = 1.0 * diagnostics->rxPreamCount;

	return 10.0 * log10( (C * pow(2.0, 17.0))/ pow(N, 2.0)) - A;
}
double get_fp_snr(dwt_rxdiag_t* diagnostics){
	return (double)(1.0 * diagnostics->firstPathAmp2) / (1.0 * diagnostics->stdNoise);
}

int16 get_confidence(double rx_pwr, double fp_pwr, double SNR){
	double conf = K1*(6 - rx_pwr + fp_pwr) + K2*SNR;
	return (int16) conf;
}

int64_t get_tof(state_data_t* state){
//	int round1 	= (b_stamps->t_rr - b_stamps->t_sp);
//	int reply1 	= (a_stamps->t_sr - a_stamps->t_rp);
//	int round2 	= (a_stamps->t_rf - a_stamps->t_sr);
//	int reply2 	= (b_stamps->t_sf - b_stamps->t_rr);
//
//	int tof 	= ((round1 - reply1) + (round2 - reply2)) / 4;
//	return tof;
	double round1 	= (double)((uint32)state->t_rr - (uint32)state->t_sp);
	double reply1 	= (double)((uint32)state->t_sr - (uint32)state->t_rp);
	double round2 	= (double)((uint32)state->t_rf - (uint32)state->t_sr);
	double reply2 	= (double)((uint32)state->t_sf - (uint32)state->t_rr);

	int64_t tof 		= (int64_t)( round1*round2 - reply1*reply2 )/(round1+round2+reply1+reply2);
	state->tof = tof;
	return tof;
}

void init_anchor_array(AnchorData* anchors, int len){
	memset(anchors, 0, len*sizeof(AnchorData)); // clear everything initially

	for(int i = 0; i < len; i++){
		anchors[i].rx_power = -INFINITY; // smalles RX power we can make
	}
}

/**
 * @brief make_mac_header use state to populate state->tx_buffer with a mac header
 * @return uint8 the size of the MAC header
 */
uint8 rtls_make_mac_header(state_data_t* state, uint8 frame_type){
	uint16_t mac_frame_ctrl = 0;
	//	uint16_t src_addr = dwt_getaddress16();
	//	uint16_t src_pan_id = dwt_getpanid();
	mac_frame_ctrl |= (uint16_t) FRAME_TYPE_DATA;//frame_type;	// indicate a frame type

	if(frame_type != FRAME_TYPE_BEACON){
		mac_frame_ctrl |= (2 << 10); 	// 16-bit destination address
		mac_frame_ctrl |= (2 << 14); 	// 16-bit source address
	}else{
		mac_frame_ctrl |= (2 << 10); 	// 16-bit destination address
		mac_frame_ctrl |= (2 << 14); 	// 16-bit source address
	}

	uint8 i = 0;
	state->tx_buffer[i++] = mac_frame_ctrl & 0xFF;
	state->tx_buffer[i++] = (mac_frame_ctrl >> 8) & 0xFF;
	state->tx_buffer[i++] = state->seq_num;

	if(frame_type == FRAME_TYPE_BEACON){
		state->tx_buffer[i++] = 0xFF; 		// dest pan id
		state->tx_buffer[i++] = 0xFF; 		// dest pan id
		state->tx_buffer[i++] = 0xFF; 		// dest address
		state->tx_buffer[i++] = 0xFF; 		// dest address
//		state->tx_buffer[i++] = 0xFF; 		// src pan id
//		state->tx_buffer[i++] = 0xFF; 		// src pan id
	}else{
		state->tx_buffer[i++] = state->transact_id & 0xFF; 			// dest pan id
		state->tx_buffer[i++] = (state->transact_id >> 8) & 0xFF; 	// dest pan id
		state->tx_buffer[i++] = state->transact_id & 0xFF; 			// dest address
		state->tx_buffer[i++] = (state->transact_id >> 8) & 0xFF; 	// dest address
	}

	state->tx_buffer[i++] = state->self_id & 0xFF; 				// src pan id
	state->tx_buffer[i++] = (state->self_id >> 8) & 0xFF; 		// src pan id
	state->tx_buffer[i++] = state->self_id & 0xFF; 				// src address
	state->tx_buffer[i++] = (state->self_id >> 8) & 0xFF; 		// src sddress

	return i;
}



AnchorData get_anchor_from_frame(uint8_t* buffer){
	AnchorData retval;
	int ind = MAC_SIZE_EXPECTED;
	if(buffer[ind++] == ANCHOR_BROADCAST){
		// if frame type matches
		retval.id = get_src_addr(buffer); // id of the sender

		memcpy(&(retval.x), buffer + ind, sizeof(retval.x));
		ind += sizeof(retval.x);

		memcpy(&(retval.y), buffer + ind, sizeof(retval.y));
		ind += sizeof(retval.y);

		memcpy(&(retval.z), buffer + ind, sizeof(retval.z));
		ind += sizeof(retval.z);
	}
	return retval;

}


void add_new_anchor(state_data_t* state, AnchorData new_anchor){
	dwt_rxdiag_t rx_diagnostics;
	dwt_readdiagnostics(&rx_diagnostics); 	// read diagnostics of the last frame
	new_anchor.is_alive 	= true;
	new_anchor.timestamp 	= HAL_GetTick(); // approximate timestamp
	new_anchor.rx_power 	= get_rx_power(&rx_diagnostics);
	new_anchor.fp_power 	= get_fp_power(&rx_diagnostics);
	new_anchor.fp_snr 		= get_fp_snr(&rx_diagnostics);

	int i;
	for(i = 0; i < state->num_anchors; i++){
		if(state->anchors[i].id == new_anchor.id){
			break;
		}
	}

	if(i >= state->num_anchors){
		state->num_anchors++; // increment to reflect the new number of anchors
		if(state->num_anchors > ANCHOR_LIST_SIZE){
			state->num_anchors = ANCHOR_LIST_SIZE;
			i = state->num_anchors - 1; // make sure we're putting new anchor at the end
		}
	}
	state->anchors[i] = new_anchor; // place anchor in list
	//sort_anchors_by_rx_power(state->anchors, state->num_anchors);

}
/**
 * @return number of anchors removed
 */
int remove_dead_anchors(AnchorData* anchors, int stop_len){
	int retval = 0;
	for(int i = 0; i < stop_len - retval; i++){
		while(!anchors[i].is_alive && retval < (stop_len-i) ){
			if(i < ((stop_len - retval) - 1)){
				// move the last element to replace the dead anchor
				memmove(anchors + i, anchors + (stop_len - retval), sizeof(anchors[i]) );
			}else{
				// last anchor in the list, so delete it
				memset(anchors + i, 0, sizeof(anchors[i]));
			}

			retval++;
		}
	}
	return retval;
}

float get_distance(point_t a, point_t b){
	double dx2 = pow((double)(b.x - a.x), 2.0);
	double dy2 = pow((double)(b.y - a.y), 2.0);
	double dz2 = pow((double)(b.z - a.z), 2.0);
	return (float)sqrt( dx2 + dy2 + dz2 );
}
/**
 * Return value meaning
 * <0 The element pointed by p1 goes before the element pointed by p2
 * 0  The element pointed by p1 is equivalent to the element pointed by p2
 * >0 The element pointed by p1 goes after the element pointed by p2
 */
int rx_power_comparator(const void* p1, const void* p2){
	float a = ((AnchorData*)p1)->rx_power;
	float b = ((AnchorData*)p2)->rx_power;
	if(a > b){
		return -1;
	}else if(b > a){
		return 1;
	}else{
		return 0;
	}
}
/**
 * Return value meaning
 * <0 The element pointed by p1 goes before the element pointed by p2
 * 0  The element pointed by p1 is equivalent to the element pointed by p2
 * >0 The element pointed by p1 goes after the element pointed by p2
 */
int rx_qual_comp(const void* p1, const void* p2){
	float rx_a = ((AnchorData*)p1)->rx_power;
	float rx_b = ((AnchorData*)p2)->rx_power;
	float fp_a = ((AnchorData*)p1)->fp_power;
	float fp_b = ((AnchorData*)p2)->fp_power;
	float snr_a = ((AnchorData*)p1)->fp_snr;
	float snr_b = ((AnchorData*)p2)->fp_snr;

	int retval = 0;

	if( (rx_a - fp_a) < (rx_b - fp_b) ){
		retval -= 1;
	}else if( (rx_a - fp_a) > (rx_b - fp_b) ){
		retval += 1;
	}

	if(snr_a > snr_b){
		retval -= 1;
	}else if(snr_a < snr_b){
		retval += 1;
	}
	return retval;
}

void sort_anchors_by_rx_power(AnchorData* anchors, int size){
	//void qsort (void* base, size_t num, size_t size, int (*comparator)(const void*,const void*));
	qsort((void*)anchors, size, sizeof(AnchorData), rx_power_comparator);
}

void sort_anchors_by_rx_qual(AnchorData* anchors, int size){
	//void qsort (void* base, size_t num, size_t size, int (*comparator)(const void*,const void*));
	qsort((void*)anchors, size, sizeof(AnchorData), rx_qual_comp);
}

void u_delay(int usec){
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
}

