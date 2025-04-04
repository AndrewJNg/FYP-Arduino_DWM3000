#define Bot_ID 'A'
#define rec_Bot_ID 'B'

#include "UWB.h"


////////////////////////////////// 5 bit fixed for protocol,  Sender ID        , Protocol messages                            , Send Time (16,17), (18,19) not sure, 20) 
static uint8_t const_receive_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE};

// static uint8_t rx_poll_msg[] =  { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0,0x00, 0x00, 0x00, 0x00, 0x00, 0x00      , 0, 0 };

static uint8_t tx_resp_msg[] =  { 0x41, 0x88, 0, 0xCA, 0xDE, Bot_ID, Bot_ID, rec_Bot_ID, rec_Bot_ID, 0xE1,0x06, 0x00, 0x00, 0x00, 0x00, 0x00      , 0, 0,  0, 0,  0, 0,  0, 0,  0, 0};
static uint8_t frame_seq_nb = 0;
static uint8_t rx_buffer[26];
static uint32_t status_reg = 0;


void setup() {
  UWB_setup();
  // /* Set expected response's delay and timeout. See NOTE 1 and 5 below.
  //  * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
  // dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
  // dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

}

void loop() {
  /* Activate reception immediately. */
  dwt_rxenable(DWT_START_RX_IMMEDIATE);


  /////////////////////////////////////////////////////////////////////////////////////////////////////
  // Part 2: Anchor successfully receive response, then compare first 10 bytes to ensure it match each other
  
  /* Poll for reception of a frame or error/timeout. See NOTE 6 below. */
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR))) {
  };

  if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
    uint32_t frame_len;

    /* Clear good RX frame event in the DW IC status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

    /* A frame has been received, read it into the local buffer. */
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (frame_len <= sizeof(rx_buffer)) {
      dwt_readrxdata(rx_buffer, frame_len, 0);
      
      /* Check that the frame is a poll sent by "SS TWR initiator" example.
       * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
      rx_buffer[ALL_MSG_SN_IDX] = 0;

  /////////////////////////////////////////////////////////////////////////////////////////////////////
  // Part 3: Compare first 10 numbers, if they match, we can move on to responding with our own message
      if (memcmp(rx_buffer, const_receive_msg, ALL_MSG_COMMON_LEN) == 0) {
        uint32_t resp_tx_time;
        int ret;
        static uint64_t T_reply_start,T_reply_end;

        /* Retrieve poll reception timestamp. */
        T_reply_start = get_rx_timestamp_u64();

        /* Compute response message transmission time. See NOTE 7 below. */
        resp_tx_time = (T_reply_start + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
        dwt_setdelayedtrxtime(resp_tx_time);

        ////////////////////////////////////////////////////////////////////////////////////
        /* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
        T_reply_end = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

        /* Write all timestamps in the final message. See NOTE 8 below. */
        resp_msg_set_ts(&tx_resp_msg[RESP_MSG_POLL_RX_TS_IDX], T_reply_start);
        resp_msg_set_ts(&tx_resp_msg[RESP_MSG_RESP_TX_TS_IDX], T_reply_end);

        /* Write and send the response message. See NOTE 9 below. */
        tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); /* Zero offset in TX buffer. */
        dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);          /* Zero offset in TX buffer, ranging. */
        ret = dwt_starttx(DWT_START_TX_DELAYED);

        /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 10 below. */
        if (ret == DWT_SUCCESS) {
          /* Poll DW IC until TX frame sent event set. See NOTE 6 below. */
          while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {
          };

          /* Clear TXFRS event. */
          dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

          /* Increment frame sequence number after transmission of the poll message (modulo 256). */
          frame_seq_nb++;
        }
      }


    }
    
      Serial.print("Received data: ");
      for (int i = 0; i < frame_len; i++) {
        Serial.print(rx_buffer[i], HEX);
        Serial.print(" ");
      }
        Serial.println(" ");
  } else {
    /* Clear RX error events in the DW IC status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
  }
}
