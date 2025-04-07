#define Bot_ID 0xCC
#define rec_Bot_ID 0xAA
#include "UWB.h"

void setup() {
  UWB_setup();
}

void loop() {

  Serial.print("DIST: ");
  Serial.println(get_UWB_Distance(Bot_ID, rec_Bot_ID));
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
