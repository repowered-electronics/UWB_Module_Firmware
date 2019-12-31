#ifndef _RTLS_H_
#define _RTLS_H_

#include "stm32f1xx_hal.h"
#include <inttypes.h>
#include "deca_device_api.h"
#include "deca_types.h"
#include "deca_regs.h"
#include "deca_spi.h"
#include "deca_mac.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#define DEVICE_MODE_TAG 	0x00
#define DEVICE_MODE_ANCHOR 	0x01

#define TX_BUFFER_SIZE 		1024

#define MAX_NUMBER_OF_ANCHORS 7

/* Index to access to sequence number of the blink frame in the tx_msg array. */
#define BLINK_FRAME_SN_IDX 	1

/* Inter-frame delay period, in milliseconds. */
#define TX_DELAY_MS 		1000

#define FRAME_LEN_MAX 		127

#define DELAY_BEFORE_TX 	0

#define SAMPLES_PER_POINT 	2
/* Change to match the device you're programming */
#define XTAL_TRIM 			15

/* Change to match which device you're programming */
#define TX_ANT_DLY 			16442
#define RX_ANT_DLY 			16442

/* weights for our confidence calculation */
#define K1 					100.0
#define K2 					1.0

#define FRAME_TYPE_DATA 	0x01
#define FRAME_TYPE_BEACON 	0x00

#define DISTANCE_FROM_TOF(tof) 	(float)(299792458.0 * (1.0*tof / (128.0*499200000.0)))

typedef struct {
	uint8_t type;
	uint8_t anchor_id;
	float distance;
	int16 confidence;
} DistanceFrame;

typedef struct AnchorTimeStamps{
	unsigned long long t_rp;
	unsigned long long t_sr;
	unsigned long long t_rf;
} AnchorTimeStamps;

typedef struct BeaconTimeStamps{
	unsigned long long t_sp;
	unsigned long long t_rr;
	unsigned long long t_sf;
	unsigned long long t_ff;
} BeaconTimeStamps;

typedef struct point {
	float x;
	float y;
	float z;
}point_t;

typedef struct AnchorData {
	uint8_t id;
	int timeout_count;
	bool is_alive;
	uint32_t timestamp;
	float x;
	float y;
	float z;
	float distance;
	float rx_power;
	float fp_power;
	float fp_snr;
}AnchorData;

typedef enum STATE {
	IDLE,
	WAIT_FOR_POLL,
	WAIT_FOR_RESPONSE,
	WAIT_FOR_FINAL,
	WAIT_FOR_DATA,
}state_t;

typedef enum TX_STATUS {
	TX_SUCCESS,
	TX_TIMEOUT
} TxStatus;

typedef enum RX_STATUS {
	RX_TIMEOUT,
	RX_DATA_FRAME_READY,
	RX_ERROR
} RxStatus;

typedef struct state_data {
	state_t state;
	uint8_t mode;
	uint16_t self_id; 		// id of self
	uint8_t channel; 		// UWB channel
	uint16_t transact_id; 	// id of the other node in the transaction
	int seq_num; 			// the sequence number
	_Bool ranging; 			// flag to indicate that we're currently ranging
	uint32_t ranging_period; // delay between ranging operations
	uint32_t timer_start; 	// used to keep track of timeouts
	uint32_t broadcast_timer;// for anchors to track broadcast intervals
	AnchorData* anchors; 	// array of anchors

	int n_range_with; 		// how many anchors to range with
	int num_anchors; 		// how many anchors are in our anchors array
	int anchor_ind; 		// current index in anchors array

	// Anchor timestamps
	unsigned long long t_rp; // receive poll
	unsigned long long t_sr; // send response
	unsigned long long t_rf; // receive final

	// TAG timestamps
	unsigned long long t_sp;
	unsigned long long t_rr;
	unsigned long long t_sf;
	unsigned long long t_ff;

	int64_t tof;
	float x;
	float y;
	float z;
	float distance;

	_Bool new_frame;
	uint8_t tx_buffer[FRAME_LEN_MAX];
	TxStatus tx_status;
	uint8_t rx_buffer[FRAME_LEN_MAX];
	RxStatus rx_status;
}state_data_t;

typedef enum RANGING_STATUS {
	RANGING_SUCCESS,
	SEND_POLL_FAILED,
	RECEIVE_POLL_FAILED,
	RECEIVE_RESPONSE_FAILED,
	SEND_FINAL_FAILED,
	RECEIVE_FINAL_FAILED,
	RECEIVE_TIMESTAMPS_FAILED
}RangingStatus;

typedef enum MESSAGE_TYPES {
	POLL,
	RESPONSE_INIT,
	SEND_FINAL,
	RESPONSE_DATA,
	ANCHOR_BROADCAST
}MessageType;


int read_rx_frame(uint8* buffer);
void rtls_fsm(state_t* state);
void send_response_init(state_data_t* state_data);
void send_response_data(state_data_t* state_data);
void send_response_final(state_data_t* state_data);
void send_anchor_broadcast(state_data_t* state);
RangingStatus range_with_anchor(uint8_t anchor_id, AnchorTimeStamps* a_stamps, BeaconTimeStamps* b_stamps, uint8* seq_num);
unsigned long long get_tx_timestamp(void);
unsigned long long get_rx_timestamp(void);
TxStatus transmit_frame(uint8* frame, int f_len, _Bool ranging);
int receive_frame(uint8* buffer, int max_len, int timeout);
double get_rx_power(dwt_rxdiag_t* diagnostics);
double get_fp_power(dwt_rxdiag_t* diagnostics);
double get_fp_snr(dwt_rxdiag_t* diagnostics);
int16 get_confidence(double rx_power, double fp_power, double snr);
int64_t get_tof(state_data_t* state);
void u_delay(int usec);
uint8 rtls_make_mac_header(state_data_t* state, uint8 frame_type);
AnchorData get_anchor_from_frame(uint8_t* buffer);
float get_distance(point_t a, point_t b);
int rx_power_comparator(const void* p1, const void* p2);
void sort_anchors_by_rx_power(AnchorData* anchors, int size);
void add_new_anchor(state_data_t* state, AnchorData new_anchor);
int remove_dead_anchors(AnchorData* anchors, int len);

#endif
