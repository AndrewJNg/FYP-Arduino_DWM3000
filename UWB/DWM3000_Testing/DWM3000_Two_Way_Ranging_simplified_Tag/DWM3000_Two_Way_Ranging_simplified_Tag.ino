#define Bot_ID 'B'
#define rec_Bot_ID 'A'

#include "UWB.h"

////////////////////////////////// 5 bit fixed for protocol,  Sender ID        , Protocol messages                            , Send Time (16,17), (18,19) not sure, 20) 
static uint8_t const_receive_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE};

// static uint8_t rx_poll_msg[] =  { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0,0x00, 0x00, 0x00, 0x00, 0x00, 0x00      , 0, 0 };
static uint8_t tx_poll_msg[] =  { 0x41, 0x88, 0, 0xCA, 0xDE, Bot_ID, Bot_ID, rec_Bot_ID, rec_Bot_ID, 0xE0,0x00, 0x05, 0x00, 0x00, 0x00, 0x00      , 0, 0 };

//                                                                                                                                       bit 16 to 19,  bit 20 to 23
// static uint8_t tx_resp_msg[] =  { 0x41, 0x88, 0, 0xCA, 0xDE, Bot_ID, Bot_ID, Bot_ID, Bot_ID, 0xE1,0x06, 0x00, 0x00, 0x00, 0x00, 0x00      ,0,0,0,0,  0,0,0,0,        0, 0};
static uint8_t frame_seq_nb = 0;
static uint8_t rx_buffer[26];
static uint32_t status_reg = 0;

static double tof;
static double distance;


void setup() {
  UWB_setup();
  
  /* Set expected response's delay and timeout. See NOTE 1 and 5 below.
   * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
  dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
  dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

}

void loop() {
    Tag_initial_send();
    Tag_waiting_for_response();
    Sleep(RNG_DELAY_MS);
  
}


// Step 1: Tag Sends Initial Message (Poll)
void Tag_initial_send() {
  
    tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);   /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);            /* Zero offset in TX buffer, ranging. */
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
}

// // Step 2: Tag Waits for Response and Computes Distance
void Tag_waiting_for_response() {
    uint32_t status_reg;
    
    // Wait for response or timeout
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {}
    frame_seq_nb++;  // Increment sequence number

    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
        process_received_message();
    } else {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    }
}


// // Helper Function: Process Received Message
void process_received_message() {
    uint32_t frame_len;
    uint32_t T_poll_start, T_poll_end, T_reply_start, T_reply_end;
    int32_t T_round, T_reply;
    float clockOffsetRatio;
    
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
    
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (frame_len <= sizeof(rx_buffer)) {
        dwt_readrxdata(rx_buffer, frame_len, 0);
        rx_buffer[ALL_MSG_SN_IDX] = 0; // Clear sequence number for validation

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

            snprintf(dist_str, sizeof(dist_str), "DIST: %3.2f m", distance);
            test_run_info((unsigned char *)dist_str);
        }
      Serial.print("Received data: ");
      for (int i = 0; i < frame_len; i++) {
        Serial.print(rx_buffer[i], HEX);
        Serial.print(" ");
      }
 
    }
}



// void loop() {
//   ///////////////////////////////////////////////////////////////////////////////////////////////////
//   // Part 1: Tag sends message to Anchor

//     // tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
//     // dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
//     // dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
//     // dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);
//     // dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

//   /* Write frame data to DW IC and prepare transmission. See NOTE 7 below. */
//   tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
//   dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
//   dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
//   dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);          /* Zero offset in TX buffer, ranging. */

//   /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
//    * set by dwt_setrxaftertxdelay() has elapsed. */
//   dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
  

//   /////////////////////////////////////////////////////////////////////////////////////////////////////
//   /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 8 below. */
//   while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {
//   };

//   /* Increment frame sequence number after transmission of the poll message (modulo 256). */
//   frame_seq_nb++;

//   if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
//     uint32_t frame_len;

//     /* Clear good RX frame event in the DW IC status register. */
//     dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

//     /* A frame has been received, read it into the local buffer. */
//     frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
//     if (frame_len <= sizeof(rx_buffer)) {
//       dwt_readrxdata(rx_buffer, frame_len, 0);

//       /* Extract the device ID from the received message. */

//       /* Check that the frame is the expected response from the companion "SS TWR responder" example.
//        * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
//       rx_buffer[ALL_MSG_SN_IDX] = 0;
//       if (memcmp(rx_buffer, const_receive_msg, ALL_MSG_COMMON_LEN) == 0) {
//         uint32_t T_poll_start, T_poll_end, T_reply_start, T_reply_end;
//         int32_t T_round, T_reply;
//         float clockOffsetRatio;

//         /* Retrieve poll transmission and response reception timestamps. See NOTE 9 below. */
//         T_poll_start = dwt_readtxtimestamplo32();
//         T_poll_end = dwt_readrxtimestamplo32();

//         /* Read carrier integrator value and calculate clock offset ratio. See NOTE 11 below. */
//         clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

//         /* Get timestamps embedded in response message. */
//         resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &T_reply_start);
//         resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &T_reply_end);

//         /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
//         T_round = T_poll_end - T_poll_start;
//         T_reply = T_reply_end - T_reply_start;

//         tof = ((T_round - T_reply * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
//         distance = tof * SPEED_OF_LIGHT;

//         /* Display computed distance on LCD. */
//         snprintf(dist_str, sizeof(dist_str), "DIST: %3.2f m", distance);
//         test_run_info((unsigned char *)dist_str);
//       }

//       // Serial.print("\nDevice ID: ");
//       // Serial.println(device_id);
//       /* Print the received message and the device ID. */
//       Serial.print("Received data: ");
//       for (int i = 0; i < frame_len; i++) {
//         Serial.print(rx_buffer[i], HEX);
//         Serial.print(" ");
//       }
//     }
//   } else {
//     /* Clear RX error/timeout events in the DW IC status register. */
//     dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
//   }

//   /* Execute a delay between ranging exchanges. */
//   Sleep(RNG_DELAY_MS);
  
// }