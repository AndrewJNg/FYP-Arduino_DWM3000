#define Bot_ID 0xCC
#define rec_Bot_ID 0xAA
#include "UWB.h"

/*
[0x41, 0x88, 0x00, 0xCA, 0xDE]             // Fixed header (5 bytes)
[Bot_ID_H, Bot_ID_L, Rec_ID_H, Rec_ID_L]   // 2-byte sender and receiver IDs
[0xE0 / 0xE1 / 0xE2]                        // Message type (start, response, final)
[0x06 to 0x01]                              // Pos (x,y,z) & Vel (x,y,z) – 6 bytes
[8 bytes]                                   // Timestamps
[2 bytes]                                   // Final chip-related bytes
*/
typedef struct {
  uint16_t sender_id;
  uint16_t receiver_id;
  uint8_t message_type;  // 0xE0 / 0xE1 / 0xE2

  int8_t position[3];  // x, y, z
  int8_t velocity[3];  // vx, vy, vz

  uint8_t timestamps[8];  // hidden from top-level, but available if needed
  uint8_t chip_bytes[2];  // reserved
} UWBMessage_t;

////////////////////////////////// 5 bit fixed for protocol,  Sender ID        , Protocol messages                            , Send Time (16,17), (18,19) not sure, 20)
static uint8_t const_receive_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, Bot_ID, Bot_ID };
// static uint8_t tx_poll_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, Bot_ID, Bot_ID, rec_Bot_ID, rec_Bot_ID, 0xE0, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0, 0 };


void setup() {
  UWB_setup();
}

/*
Level 5 
int get_UWB_Distance(&current_device_address, &communication_device_address, displacement, velocity)  //Obtain the distance in metre between this device and targeted communication device
int scan_for_messages(&current_device_address, &communication_device_address)   //if it receive a message, reply to them (if it ask from current_device_address) or check if these is any new device on network, 
        if new device detected, return 1

Level 4






*/
// UWBMessage_t myMessage;
void loop() {

  Serial.print("DIST: ");
  Serial.println(get_UWB_Distance(0xCCCC, 0xAAAA));
  Sleep(500);
}

double get_UWB_Distance(uint16_t sender_id, uint16_t receiver_id) {
  static double distance = 0;
  Tag_set_send_mode(sender_id, receiver_id);
  distance = Tag_waiting_for_response(sender_id, receiver_id);
  return distance;
}


// Step 1: Tag Sends Initial Message (Poll)
void Tag_set_send_mode(uint16_t sender_id, uint16_t receiver_id) {
  // Set the receive timeout for the response message in microseconds.
  dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

  // Prepare message to be sent out
  uint8_t tx_msg[18];
  int8_t pos[3] = { 1, 2, 3 };
  int8_t vel[3] = { 4, 5, 6 };
  generate_msg(tx_msg, frame_seq_nb, receiver_id, sender_id, pos, vel, 0xE0, NULL);

  ////// Setup to send //////
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);  // Clear TX frame sent event bit in status register to prepare for transmission
  dwt_writetxdata(sizeof(tx_msg), tx_msg, 0);                   // Write poll message data into TX buffer starting at offset 0

  // Configure the TX frame control:
  // - Frame length set to the size of the poll message.
  // - Offset 0 in TX buffer.
  // - Enable ranging bit (important for distance measurement).
  dwt_writetxfctrl(sizeof(tx_msg), 0, 1);

  // Start transmission immediately and expect a response.
  dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
}

// // Step 2: Tag Waits for Response and Computes Distance
double Tag_waiting_for_response(uint16_t sender_id, uint16_t receiver_id) {
  static double distance = 0;
  // Wait for response or timeout
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {
  };

  if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
    distance = Tag_process_received_message(sender_id, receiver_id);
    return distance;
  } else {
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    return -1;
  }
}


// Helper Function: Process Received Message
double Tag_process_received_message(uint16_t sender_id, uint16_t receiver_id) {
  static double distance = 0;
  uint32_t frame_len;
  uint32_t T_poll_start, T_poll_end, T_reply_start, T_reply_end;
  int32_t T_round, T_reply;
  float clockOffsetRatio;
  static double tof;

  frame_seq_nb++;  // Increment sequence number

  ////////////////////////////////////////////////////////////////////////
  // Clear good RX frame event in the DW IC status register.
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

  // Read the received frame into the local buffer.
  frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
  if (frame_len <= sizeof(rx_buffer)) {
    dwt_readrxdata(rx_buffer, frame_len, 0);
    rx_buffer[ALL_MSG_SN_IDX] = 0;  // Clear sequence number for validation

    if (memcmp(rx_buffer, const_receive_msg, ALL_MSG_COMMON_LEN + 2) == 0) {  // add 2 for receiver message including bot specific msg
      T_poll_start = dwt_readtxtimestamplo32();
      T_poll_end = dwt_readrxtimestamplo32();
      clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

      resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &T_reply_start);
      resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &T_reply_end);

      T_round = T_poll_end - T_poll_start;
      T_reply = T_reply_end - T_reply_start;
      tof = ((T_round - T_reply * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
      distance = tof * SPEED_OF_LIGHT;

      // snprintf(dist_str, sizeof(dist_str), "DIST: %3.2f m", distance);
      // test_run_info((unsigned char *)dist_str);
    } else 
    {
      Serial.println("Ignored");
      // distance = -1;
    }
    printRxBuffer(rx_buffer, frame_len);
  } else distance = -1;

  return distance;
}

void generate_msg(uint8_t *tx_msg,
                  uint8_t frame_seq_nb,
                  uint16_t sender_id,
                  uint16_t receiver_id,
                  int8_t position[3],
                  int8_t velocity[3],
                  uint8_t message_type,
                  uint8_t *timestamps)  // can be NULL
{
  tx_msg[0] = 0x41;
  tx_msg[1] = 0x88;
  tx_msg[2] = frame_seq_nb;
  tx_msg[3] = 0xCA;
  tx_msg[4] = 0xDE;

  tx_msg[5] = (sender_id >> 8) & 0xFF;  // sender_id high byte
  tx_msg[6] = sender_id & 0xFF;         // sender_id low byte

  tx_msg[7] = (receiver_id >> 8) & 0xFF;  // receiver_id high byte
  tx_msg[8] = receiver_id & 0xFF;         // receiver_id low byte

  tx_msg[9] = position[0];
  tx_msg[10] = position[1];
  tx_msg[11] = position[2];

  tx_msg[12] = velocity[0];
  tx_msg[13] = velocity[1];
  tx_msg[14] = velocity[2];

  tx_msg[15] = message_type;

  // Default timestamps to 0
  tx_msg[16] = 0x00;
  tx_msg[17] = 0x00;

  // Total: 18 bytes — you can extend this if you're using all 8 timestamps/chip bytes
}