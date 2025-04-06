#define Bot_ID 0xAA
#define rec_Bot_ID 0xBB
#include "UWB.h"

/*
[0x41, 0x88, 0x00, 0xCA, 0xDE]             // Fixed header (5 bytes)
[Bot_ID_H, Bot_ID_L, Rec_ID_H, Rec_ID_L]   // 2-byte sender and receiver IDs
[0xE0 / 0xE1 / 0xE2]                        // Message type (start, response, final)
[0x06 to 0x01]                              // Pos (x,y,z) & Vel (x,y,z) – 6 bytes
[8 bytes]                                   // Timestamps
[2 bytes]                                   // Final chip-related bytes
*/
////////////////////////////////// 5 bit fixed for protocol,  Sender ID        , Protocol messages                            , Send Time (16,17), (18,19) not sure, 20)
static uint8_t const_receive_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, Bot_ID, Bot_ID };
// static uint8_t tx_resp_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, rec_Bot_ID, rec_Bot_ID, Bot_ID, Bot_ID, 0xE1, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };



void setup() {
  UWB_setup();
}

void loop() {
  Anchor_set_receive_mode();      // Actively listen to messages
  Anchor_waiting_for_response(0xAAAA);  // Respond to messages if it is called
}
void Anchor_set_receive_mode() {
  // Activate reception immediately (no timeout).
  dwt_setrxtimeout(0);
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
}
void Anchor_waiting_for_response(uint16_t sender_id) {
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
    uint16_t receiver_id = ((uint16_t)rx_buffer[8] << 8) | rx_buffer[7];

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
