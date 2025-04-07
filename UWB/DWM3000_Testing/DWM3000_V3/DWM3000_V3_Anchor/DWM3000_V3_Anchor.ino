#define Bot_ID 0xAA
#include "UWB.h"



void setup() {
  UWB_setup();
}

void loop() {
  Anchor_waiting_for_response(Bot_ID);  // Respond to messages if it is called
}

void Anchor_waiting_for_response(uint16_t sender_id) {
  // Set as Anchor and to activate reception immediately (no timeout).
  dwt_setrxtimeout(0);
  dwt_rxenable(DWT_START_RX_IMMEDIATE);

  // Wait for a received frame or error/timeout.
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR))) {
  }

  // If a valid frame is received, process the message.
  if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
    Anchor_process_received_message(sender_id);
  } else {
    // Clear RX error events in the DW IC status register.
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
  }
}

void Anchor_process_received_message(uint16_t sender_id) {
  uint32_t frame_len;

  ////////////////////////////////////////////////////////////////////////
  // Clear good RX frame event in the DW IC status register.
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

  // Read the received frame into the local buffer.
  frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
  if (frame_len <= sizeof(rx_buffer)) {
    memset(rx_buffer, 0, sizeof(rx_buffer));  // <-- Clear buffer first

    dwt_readrxdata(rx_buffer, frame_len, 0);
    rx_buffer[ALL_MSG_SN_IDX] = 0;  // Clear sequence number for validation
    uint16_t receiver_id = ((uint16_t)rx_buffer[7] << 8) | rx_buffer[8];
    // uint16_t receiver_id = ((uint16_t)rx_buffer[8] << 8) | rx_buffer[7];

    // Check if the received message matches the expected format.
    if (memcmp(rx_buffer, const_receive_msg, ALL_MSG_COMMON_LEN + 2) == 0) {
      uint32_t resp_tx_time;
      int ret;
      static uint64_t T_reply_start, T_reply_end;

      // Retrieve poll reception timestamp.
      T_reply_start = get_rx_timestamp_u64();

      // Compute response transmission time.
      resp_tx_time = (T_reply_start + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
      dwt_setdelayedtrxtime(resp_tx_time);

      // Calculate the final response TX timestamp.
      T_reply_end = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

      uint8_t tx_msg[26];
      int8_t pos[3] = { 6, 5, 4 };
      int8_t vel[3] = { 3, 2, 1 };
      generate_msg(tx_msg, frame_seq_nb, receiver_id, sender_id, pos, vel, 0xE1, NULL);

      // Store timestamps in the response message.
      resp_msg_set_ts(&tx_msg[RESP_MSG_POLL_RX_TS_IDX], T_reply_start);
      resp_msg_set_ts(&tx_msg[RESP_MSG_RESP_TX_TS_IDX], T_reply_end);

      // Prepare and send the response message.
      // tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
      dwt_writetxdata(sizeof(tx_msg), tx_msg, 0);
      dwt_writetxfctrl(sizeof(tx_msg), 0, 1);
      ret = dwt_starttx(DWT_START_TX_DELAYED);

      // If transmission is successful, wait for TX event and clear it.
      if (ret == DWT_SUCCESS) {
        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {
        }
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

        // Increment frame sequence number for next message.
        frame_seq_nb++;
      }
    } else Serial.println("Ignored");
  }

  printRxBuffer(rx_buffer, frame_len);
}

