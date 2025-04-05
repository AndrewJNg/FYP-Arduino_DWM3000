#define Bot_ID 'B'
#define rec_Bot_ID 'A'
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
    uint8_t message_type; // 0xE0 / 0xE1 / 0xE2

    int8_t position[3];   // x, y, z
    int8_t velocity[3];   // vx, vy, vz

    uint8_t timestamps[8]; // hidden from top-level, but available if needed
    uint8_t chip_bytes[2]; // reserved
} UWBMessage_t;

////////////////////////////////// 5 bit fixed for protocol,  Sender ID        , Protocol messages                            , Send Time (16,17), (18,19) not sure, 20)
static uint8_t const_receive_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE };
static uint8_t tx_poll_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, Bot_ID, Bot_ID, rec_Bot_ID, rec_Bot_ID, 0xE0, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0, 0 };


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
  // myMessage.sender_id = 'A' << 8 | 'A';    // 'AA'
  // myMessage.receiver_id = 'B' << 8 | 'B';  // 'BB'
  // // myMessage.message_type = 0xE1;

  // myMessage.position[0] = 6;  // x
  // myMessage.position[1] = 5;  // y
  // myMessage.position[2] = 4;  // z

  // myMessage.velocity[0] = 3;  // vx
  // myMessage.velocity[1] = 2;  // vy
  // myMessage.velocity[2] = 1;  // vz
  
  Serial.print("DIST: ");
  Serial.println(get_UWB_Distance());
  Sleep(500);
}

double get_UWB_Distance() {
  static double distance = 0;
  Tag_set_send_mode();
  distance = Tag_waiting_for_response();
  return distance;
}


// Step 1: Tag Sends Initial Message (Poll)
void Tag_set_send_mode() {
  // Set the receive timeout for the response message in microseconds.
  dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

  // Update the sequence number in the poll message to maintain unique frames.
  tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;

  // Clear the TX frame sent event bit in the status register to prepare for transmission.
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

  // Write the poll message data into the TX buffer starting at offset 0.
  dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);

  // Configure the TX frame control:
  // - Frame length set to the size of the poll message.
  // - Offset 0 in TX buffer.
  // - Enable ranging bit (important for distance measurement).
  dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);

  // Start transmission immediately and expect a response.
  dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
}
static double distance = 0;
// // Step 2: Tag Waits for Response and Computes Distance
double Tag_waiting_for_response() {
  // Wait for response or timeout
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {
  };

  if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
    distance = Tag_process_received_message();
    return distance;
  } else {
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    return -1;
  }
}


// Helper Function: Process Received Message
double Tag_process_received_message() {
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

    if (memcmp(rx_buffer, const_receive_msg, ALL_MSG_COMMON_LEN) == 0) {
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
    }


    printRxBuffer(rx_buffer, frame_len);
  } else distance = -1;

  return distance;
}
