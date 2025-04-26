
#include "dw3000.h"
#include "SPI.h"

extern SPISettings _fastSPI;

double get_UWB_Distance(uint16_t sender_id, uint16_t receiver_id);
void Tag_set_send_mode(uint16_t sender_id, uint16_t receiver_id);
double Tag_waiting_for_response(uint16_t sender_id, uint16_t receiver_id);
double Tag_process_received_message(uint16_t sender_id, uint16_t receiver_id);

void Anchor_waiting_for_response(uint16_t sender_id);
void Anchor_process_received_message(uint16_t sender_id);

void setTransmitData(int length, uint8_t *buffer, int ranging, int fcs);
int startTransmit(bool delayed, bool wait4resp);

// #define PIN_RST 33
// #define PIN_IRQ 40  // Pin IRQ is useless for now, set to arbitary pin to not cause conflict
// #define PIN_SS 32


// Default settings
#define PIN_RST 27
#define PIN_IRQ 34
#define PIN_SS 4

/*
// If we resolder new boards with DRV8333 motor driver
#define PIN_RST 15
#define PIN_IRQ 13 
#define PIN_SS 5
*/

#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

// #define TX_ANT_DLY 16385
// #define RX_ANT_DLY 16385

#define ALL_MSG_COMMON_LEN 5
#define ALL_MSG_SN_IDX 2

#define RESP_MSG_POLL_RX_TS_IDX 16
#define RESP_MSG_RESP_TX_TS_IDX 20

#define POLL_RX_TO_RESP_TX_DLY_UUS 450
#define POLL_TX_TO_RESP_RX_DLY_UUS 240
#define RESP_RX_TIMEOUT_UUS 400

static uint8_t frame_seq_nb = 0;
static uint8_t rx_buffer[28];
static uint32_t status_reg = 0;

// extern dwt_txconfig_t txconfig_options;
dwt_txconfig_t txconfig_options2 = {
  0x34,       /* PG delay. */
  0xffffffff, /* TX power. */
  0x0         /*PG count*/
};


// #define CONFIG_OPTION_01
/* Default communication configuration. We use default non-STS DW mode. */


//// Working Channel 5 code
static dwt_config_t config = {
  5,                /* Channel number. */
  DWT_PLEN_32,      /* Preamble length. Used in TX only. */
  DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
  9,                /* TX preamble code. Used in TX only. */
  9,                /* RX preamble code. Used in RX only. */
  1,                /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
  DWT_BR_6M8,       /* Data rate. */
  DWT_PHRMODE_STD,  /* PHY header mode. */
  DWT_PHRRATE_STD,  /* PHY header rate. */
  (33 + 8 - 8),     /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
  DWT_STS_MODE_OFF, /* STS disabled */
  DWT_STS_LEN_64,   /* STS length see allowed values in Enum dwt_sts_lengths_e */
  DWT_PDOA_M0       /* PDOA mode off */
};




void UWB_setup() {
  UART_init();

  // 1. HARD RESET
  pinMode(PIN_RST, OUTPUT);
  digitalWrite(PIN_RST, LOW);
  delay(10);  // hold in reset
  digitalWrite(PIN_RST, HIGH);
  delay(10);  // let chip wake

  // delay(3000);  // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)
  // 2. BEGIN SPI
  _fastSPI = SPISettings(8000000L, MSBFIRST, SPI_MODE0);
  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);
  delay(2);  // short delay

  // dwt_softreset();
  //  _fastSPI = SPISettings(16000000L, MSBFIRST, SPI_MODE0);

  // 3. SOFTWARE RESET (optional but good)
  dwt_softreset();
  delay(2);  // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

  // while (!dwt_checkidlerc())  // Need to make sure DW IC is in IDLE_RC before proceeding
  // {
  //   UART_puts("IDLE FAILED\r\n");
  //   while (1)
  //     ;
  // }

  do  // Need to make sure DW IC is in IDLE_RC before proceeding
  {
    UART_puts("IDLE FAILED\r\n");
    // while (1)
    // ;
    dwt_softreset();
    delay(500);  // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)
  } while (!dwt_checkidlerc() || dwt_initialise(DWT_DW_INIT) == DWT_ERROR);

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
    UART_puts("INIT FAILED\r\n");
    while (1)
      ;
  }

  // Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards.
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  /* Configure DW IC. See NOTE 6 below. */
  if (dwt_configure(&config))  // if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device
  {
    UART_puts("CONFIG FAILED\r\n");
    while (1)
      ;
  }

  /* Configure the TX spectrum parameters (power, PG delay and PG count) */
  dwt_configuretxrf(&txconfig_options2);

  /* Apply default antenna delay value. See NOTE 2 below. */
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);

  /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug, and also TX/RX LEDs
   * Note, in real low power applications the LEDs should not be used. */
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

  // /* Set expected response's delay and timeout. See NOTE 1 and 5 below.
  //  * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
  dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
  dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

  Serial.println("Range Tag");
  Serial.println("Setup over........");
}

void printRxBuffer(uint8_t *buffer, uint16_t length) {
  Serial.print("Received data: ");
  for (uint16_t i = 0; i < length; i++) {
    if (buffer[i] < 0x10) Serial.print("0");  // Pad single-digit hex with zero
    Serial.print(buffer[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

//////////////////////////////////////////////////////////////////////////////////////
// Function to generate new UWB messages to be sent over
/*
[0x41, 0x88, 0x00, 0xCA, 0xDE]             // Fixed header (5 bytes)
[Bot_ID_H, Bot_ID_L, Rec_ID_H, Rec_ID_L]   // 2-byte sender and receiver IDs
[0xE0 / 0xE1 / 0xE2]                        // Message type (start, response, final)
[0x06 to 0x01]                              // Pos (x,y,z) & Vel (x,y,z) – 6 bytes
[8 bytes]                                   // Timestamps
[2 bytes]                                   // Final chip-related bytes
*/
static uint8_t const_receive_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 0x00, Bot_ID };
// static uint8_t tx_resp_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, rec_Bot_ID, rec_Bot_ID, Bot_ID, Bot_ID, 0xE1, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

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

  tx_msg[9] = message_type;

  tx_msg[10] = position[0];
  tx_msg[11] = position[1];
  tx_msg[12] = position[2];

  tx_msg[13] = velocity[0];
  tx_msg[14] = velocity[1];
  tx_msg[15] = velocity[2];


  // Default timestamps to 0
  tx_msg[16] = 0x00;
  tx_msg[17] = 0x00;

  // Total: 18 bytes — you can extend this if you're using all 8 timestamps/chip bytes
}

double get_UWB_Distance(uint16_t sender_id, uint16_t receiver_id) {
  static double distance = 0;
  Tag_set_send_mode(sender_id, receiver_id);
  distance = Tag_waiting_for_response(sender_id, receiver_id);
  return distance;
}

void setTransmitData(int length, uint8_t *buffer, int ranging, int fcs) {
  // Write data to TX buffer
  dwt_writetxdata(length, buffer, 0);

  // Frame control settings
  uint32_t ranging_flag = (ranging ? 1 : 0);
  dwt_writetxfctrl(length, 0, ranging_flag);
}

int startTransmit(bool delayed, bool wait4resp) {
  uint8_t tx_flag = 0;
  if (delayed)
    tx_flag |= DWT_START_TX_DELAYED;
  else
    tx_flag |= DWT_START_TX_IMMEDIATE;

  if (wait4resp)
    tx_flag |= DWT_RESPONSE_EXPECTED;

  return dwt_starttx(tx_flag);
}




// Step 1: Tag Sends Initial Message (Poll)
void Tag_set_send_mode(uint16_t sender_id, uint16_t receiver_id) {
  // Set the receive timeout for the response message in microseconds.
  dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);  // Clear TX frame sent event bit in status register

  // Prepare message to be sent out
  uint8_t tx_msg[18];
  int8_t pos[3] = { 1, 2, 3 };
  int8_t vel[3] = { 4, 5, 6 };
  generate_msg(tx_msg, frame_seq_nb, receiver_id, sender_id, pos, vel, 0xE0, NULL);

  setTransmitData(sizeof(tx_msg), tx_msg, 1, 1);  // ranging = 1, fcs = 1
  startTransmit(false, true);                     // immediate TX, expect response
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
    } else {
      Serial.println("Ignored");
      // distance = -1;
    }
    printRxBuffer(rx_buffer, frame_len);
  } else distance = -1;

  return distance;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

      if (rx_buffer[9] == 0xE0) {
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

        uint8_t tx_msg[22];
        int8_t pos[3] = { 6, 5, 4 };
        int8_t vel[3] = { 3, 2, 1 };
        generate_msg(tx_msg, frame_seq_nb, receiver_id, sender_id, pos, vel, 0xE1, NULL);

        // Store timestamps in the response message.
        resp_msg_set_ts(&tx_msg[RESP_MSG_POLL_RX_TS_IDX], T_reply_end - T_reply_start);
        // resp_msg_set_ts(&tx_msg[RESP_MSG_RESP_TX_TS_IDX], T_reply_end);

        // Prepare and send the response message
        setTransmitData(sizeof(tx_msg), tx_msg, 1, 1);  // ranging = 1, fcs = 1
        // startTransmit(true, false);                     // delayed TX, no response expected

        startTransmit(true, true);  // delayed TX, response expected

        // // Wait for TX confirmation
        // while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {
        //   // Optional timeout mechanism to avoid infinite loop
        // }

        // // Clear the TXFRS flag
        // dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

        // // Re-enable receiver
        // dwt_rxenable(DWT_START_RX_IMMEDIATE);
        Serial.println("Second message sent");
        dwt_setrxtimeout(0);
        // dwt_rxenable(DWT_START_RX_IMMEDIATE);

      } else if (rx_buffer[9] == 0xE2) {  // Check for Final message type

        // static double distance = 0;
        // uint32_t frame_len;
        // uint32_t T_poll_start2, T_poll_end2, T_reply_start, T_reply_end;
        // uint32_t T_reply1, T_round1;
        // uint32_t T_reply2, T_round2;

        // float clockOffsetRatio;
        // static double tof;

        // T_poll_start2 = dwt_readtxtimestamplo32();
        // T_poll_end2 = dwt_readrxtimestamplo32();
        // clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

        // // resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &T_reply_start);
        // // resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &T_reply_end);
        // resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &T_reply1);
        // resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &T_round1);
        // resp_msg_get_ts(&rx_buffer[24], &T_reply2);

        // T_round2 = T_poll_end2 - T_poll_start2;
        // // T_reply = T_reply_end - T_reply_start;
        // tof = ((T_round2 - T_reply2 * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
        // distance = tof * SPEED_OF_LIGHT;

        // snprintf(dist_str, sizeof(dist_str), "DIST: %3.2f m", distance);
        // test_run_info((unsigned char *)dist_str);
        // Serial.println("Second message sent");

        Serial.println("Final message received");

        // STEP 1: Compute reply timestamps and clock ratio
        T_poll_start = dwt_readtxtimestamplo32();
        T_poll_end = dwt_readrxtimestamplo32();
        clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

        // STEP 2: Read timestamps from anchor's response (rx_buffer)
        resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &T_reply);  // T_reply = round2 - reply2

        // resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &T_reply_start);
        // resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &T_reply_end);
        // resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &T_reply);
        // resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &T_reply_end);

        // STEP 3: Calculate round-trip and distance
        T_round = T_poll_end - T_poll_start;
        // T_reply = T_reply_end - T_reply_start;
        tof = ((T_round - T_reply * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
        distance = tof * SPEED_OF_LIGHT;
        
      // Serial.println("Final sent: ");
      // printRxBuffer(tx_msg, sizeof(tx_msg));
      // Serial.println(ret);
      snprintf(dist_str, sizeof(dist_str), "DIST: %3.2f m", distance);
      test_run_info((unsigned char *)dist_str);










        // Optional: extract and interpret Final message payload
        // uint64_t final_ts_data = 0;
        // memcpy(&final_ts_data, &rx_buffer[ALL_MSG_COMMON_LEN + 3], sizeof(uint64_t));

        // uint32_t T_poll_start = (uint32_t)(final_ts_data & 0xFFFFFFFFUL);
        // uint32_t T_reply_end = (uint32_t)(final_ts_data >> 32);

        // Serial.print("T_poll_start: ");
        // Serial.println(T_poll_start);
        // Serial.print("T_reply_end: ");
        // Serial.println(T_reply_end);


        // startTransmit(true, true);  // delayed TX, response expected

        // while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {
        // };

        // if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
        //   dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
        //   // Read the received frame into the local buffer.
        //   frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
        //   if (frame_len <= sizeof(rx_buffer)) {
        //     dwt_readrxdata(rx_buffer, frame_len, 0);

        //     rx_buffer[ALL_MSG_SN_IDX] = 0;  // Clear sequence number for validation
        //     if (memcmp(rx_buffer, const_receive_msg, ALL_MSG_COMMON_LEN + 2) == 0) {
        //     }
        //     // distance = Tag_process_received_message(sender_id, receiver_id);
        //     // return distance;
        //   }
        // } else {
        //   dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        //   // return -1;
        // }
      } else Serial.println("Ignored msg");
    } else Serial.println("Ignored");
  }

  printRxBuffer(rx_buffer, frame_len);
}
