
// https://github.com/Ai-Thinker-Open/STM32F103-BU0x_SDK/blob/master/Components/Examples/examples/ex_03a_tx_wait_resp/tx_wait_resp.c
#include "dw3000.h"
#include "SPI.h"

// Default settings
// #define PIN_RST 27
// #define PIN_IRQ 34
// #define PIN_SS 4

extern SPISettings _fastSPI;

double get_UWB_Distance(uint16_t sender_id, uint16_t receiver_id);
void Tag_set_send_mode(uint16_t sender_id, uint16_t receiver_id, int8_t position[3], int8_t velocity[3]);
double Tag_waiting_for_response(uint16_t sender_id, uint16_t receiver_id);
double Tag_process_received_message(uint16_t sender_id, uint16_t receiver_id);

void Anchor_waiting_for_response(uint16_t sender_id, int8_t pos[3], int8_t vel[3]);
void Anchor_process_received_message(uint16_t sender_id, int8_t pos[3], int8_t vel[3]);

void setTransmitData(int length, uint8_t *buffer, int ranging, int fcs);
int startTransmit(bool delayed, bool wait4resp);


#define TX_ANT_DLY 16370
#define RX_ANT_DLY 16370

// #define TX_ANT_DLY 16385
// #define RX_ANT_DLY 16385

#define ALL_MSG_COMMON_LEN 5
#define ALL_MSG_SN_IDX 2

#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14

#define POLL_RX_TO_RESP_TX_DLY_UUS 500
#define POLL_TX_TO_RESP_RX_DLY_UUS 240
#define RESP_RX_TIMEOUT_UUS 400

static uint8_t frame_seq_nb = 0;
static uint8_t rx_buffer[34];
static uint32_t status_reg = 0;


int8_t read_position[3] = { 0, 0, 0 };
int8_t read_velocity[3] = { 0, 0, 0 };

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// extern dwt_txconfig_t txconfig_options;
dwt_txconfig_t txconfig_options2 = {
  0x34,       /* PG delay. */
  0xffffffff, /* TX power. */
  0x0         /*PG count*/
};

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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void UWB_setup(int PIN_RST, int PIN_IRQ, int PIN_SS) {
  UART_init();

  // 1. HARD RESET
  pinMode(PIN_RST, OUTPUT);
  digitalWrite(PIN_RST, LOW);
  delay(10);  // hold in reset
  digitalWrite(PIN_RST, HIGH);
  delay(10);  // let chip wake

  // 2. BEGIN SPI
  _fastSPI = SPISettings(8000000L, MSBFIRST, SPI_MODE0);
  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);
  delay(2);  // short delay

  // 3. SOFTWARE RESET (optional but good)
  dwt_softreset();
  delay(2);  // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

  // Need to make sure DW IC is in IDLE_RC before proceeding, if it fails, do a soft reset and try again
  do {
    UART_puts("IDLE FAILED\r\n");
    dwt_softreset();
    delay(500);  // Extra time given for DW3000 to start up again
  } while (!dwt_checkidlerc() || dwt_initialise(DWT_DW_INIT) == DWT_ERROR);

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
    UART_puts("INIT FAILED\r\n");
    while (1)
      ;
  }

  //// Optionally Configure GPIOs to show TX/RX activity. ////
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

  //// Optionally enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards. See Note 10 below. /////
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /* Configure DW IC. See NOTE 6 below. */
  if (dwt_configure(&config))  // if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device
  {
    UART_puts("CONFIG FAILED\r\n");
    while (1)
      ;
  }

  /* Configure the TX spectrum parameters (power, PG delay and PG count) */
  dwt_configuretxrf(&txconfig_options2);

  // /* Set expected response's delay and timeout. See NOTE 1 and 5 below.
  //  * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
  dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
  dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

  /* Apply default antenna delay value. See NOTE 2 below. */
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);

  Serial.println("Range Tag");
  Serial.println("Setup over........");
}

void printRxBuffer(uint8_t *buffer, uint16_t length) {
  buffer[2] = frame_seq_nb;
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
                  uint8_t message_type)  // can be NULL
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double get_UWB_Distance(uint16_t sender_id, uint16_t receiver_id, int8_t pos[3], int8_t vel[3]) {
  static double distance = 0;
  Tag_set_send_mode(sender_id, receiver_id, pos, vel);
  distance = Tag_waiting_for_response(sender_id, receiver_id);
  return distance;
}


// Step 1: Tag Sends Initial Message (Poll)
void Tag_set_send_mode(uint16_t sender_id, uint16_t receiver_id, int8_t pos[3], int8_t vel[3]) {
  // Set the receive timeout for the response message in microseconds.
  dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);  // Clear TX frame sent event bit in status register

  // Prepare message to be sent out
  frame_seq_nb++;  // Increment sequence number
  uint8_t tx_msg[18];
  generate_msg(tx_msg, frame_seq_nb, receiver_id, sender_id, 0xE0);
  tx_msg[10] = pos[0];
  tx_msg[11] = pos[1];
  tx_msg[12] = pos[2];

  tx_msg[13] = vel[0];
  tx_msg[14] = vel[1];
  tx_msg[15] = vel[2];

  setTransmitData(sizeof(tx_msg), tx_msg, 1, 1);  // ranging = 1, fcs = 1
  startTransmit(false, true);                     // immediate TX, expect response
}

// // Step 2: Tag Waits for Response and Computes Distance
double Tag_waiting_for_response(uint16_t sender_id, uint16_t receiver_id) {
  static double distance = 0;
  // Wait for response or timeout

  // dwt_setrxtimeout(0);
  // dwt_rxenable(DWT_START_RX_IMMEDIATE);

  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {
  };

  if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
    distance = Tag_process_received_message(sender_id, receiver_id);

    // Clear good RX frame event in the DW IC status register.
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

    return distance;
  } else {
    // Clear RX error/timeout events in the DW3000 status register.
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    return -1;
  }
}


// Helper Function: Process Received Message
double Tag_process_received_message(uint16_t sender_id, uint16_t receiver_id) {
  static double distance = 0;
  uint32_t frame_len;
  uint32_t T_poll_start, T_poll_end, T_reply_start, T_reply_end;
  uint32_t T_round;
  uint32_t T_reply;

  float clockOffsetRatio;
  static double tof;

  // frame_seq_nb++;  // Increment sequence number

  ////////////////////////////////////////////////////////////////////////
  // Clear good RX frame event and TX frame sent in the DW3000 status register.
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_TXFRS_BIT_MASK);

  // Read the received frame into the local buffer.
  frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
  if (frame_len <= sizeof(rx_buffer)) {
    memset(rx_buffer, 0, sizeof(rx_buffer));  // <-- Clear buffer first

    dwt_readrxdata(rx_buffer, frame_len, 0);
    rx_buffer[ALL_MSG_SN_IDX] = 0;  // Clear sequence number for validation

    // Check if the received message matches the expected format.
    if (memcmp(rx_buffer, const_receive_msg, ALL_MSG_COMMON_LEN + 2) == 0) {

      uint32_t resp_tx_time;
      int ret;
      static uint64_t T_reply_start, T_reply_end;

      // Retrieve poll reception timestamp.
      T_reply_start = get_rx_timestamp_u64();

      // Compute response transmission time.
      resp_tx_time = (T_reply_start + (POLL_RX_TO_RESP_TX_DLY_UUS * 4 * UUS_TO_DWT_TIME)) >> 8;
      dwt_setdelayedtrxtime(resp_tx_time);

      // Calculate the final response TX timestamp.
      T_reply_end = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

      uint8_t tx_msg[24];
      generate_msg(tx_msg, frame_seq_nb, receiver_id, sender_id, 0xE2);

      /////////////////////////////////////////////////////////////////////////////////////////////////////////

      // STEP 1: Compute reply timestamps and clock ratio
      T_poll_start = dwt_readtxtimestamplo32();
      T_poll_end = dwt_readrxtimestamplo32();
      clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

      uint32_t T_reply1;
      // STEP 2: Read timestamps from anchor's response (rx_buffer)
      resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX + 6], &T_reply1);  // T_reply = round2 - reply2

      resp_msg_set_ts(&tx_msg[RESP_MSG_POLL_RX_TS_IDX], T_reply1);
      resp_msg_set_ts(&tx_msg[RESP_MSG_RESP_TX_TS_IDX], T_poll_end - T_poll_start);
      resp_msg_set_ts(&tx_msg[RESP_MSG_RESP_TX_TS_IDX + 4], T_reply_end - T_reply_start);

      // STEP 3: Calculate round-trip and distance
      T_round = T_poll_end - T_poll_start;
      // T_reply = T_reply_end - T_reply_start;
      tof = ((T_round - T_reply1 * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
      distance = tof * SPEED_OF_LIGHT;

      // Prepare and send the response message
      setTransmitData(sizeof(tx_msg), tx_msg, 1, 1);  // ranging = 1, fcs = 1
      ret = startTransmit(true, false);               // delayed TX, no response expected
      if (ret != DWT_SUCCESS) Serial.println("Failed to schedule delayed TX, too late!");


      read_position[0] = rx_buffer[10];
      read_position[1] = rx_buffer[11];
      read_position[2] = rx_buffer[12];

      read_velocity[0] = rx_buffer[13];
      read_velocity[1] = rx_buffer[14];
      read_velocity[2] = rx_buffer[15];

      /////////////////////////////////////////////////////////////////////////////////////////////////

    } else {
      Serial.println("Ignored");
      // distance = -1;
    }
    printRxBuffer(rx_buffer, frame_len);
  } else distance = -1;

  return distance;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Anchor_waiting_for_response(uint16_t sender_id, int8_t pos[3], int8_t vel[3]) {
  // Set as Anchor and to activate reception immediately (no timeout).
  dwt_setrxtimeout(0);
  dwt_rxenable(DWT_START_RX_IMMEDIATE);

  // Wait for a received frame or error/timeout.
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR))) {
  }

  // If a valid frame is received, process the message.
  if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
    Anchor_process_received_message(sender_id, pos, vel);

    // Clear good RX frame event in the DW IC status register.
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
  } else {
    // Clear RX error events in the DW IC status register.
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
  }
}

void Anchor_process_received_message(uint16_t sender_id, int8_t pos[3], int8_t vel[3]) {
  uint32_t frame_len;

  ////////////////////////////////////////////////////////////////////////
  // Clear good RX frame event in the DW IC status register.
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

  // Read the received frame into the local buffer.
  frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;

  if (frame_len <= sizeof(rx_buffer)) {
    memset(rx_buffer, 0, sizeof(rx_buffer));  // <-- Clear buffer first

    dwt_readrxdata(rx_buffer, frame_len, 0);
    frame_seq_nb = rx_buffer[ALL_MSG_SN_IDX];
    rx_buffer[ALL_MSG_SN_IDX] = 0;  // Clear sequence number for validation
    uint16_t receiver_id = ((uint16_t)rx_buffer[7] << 8) | rx_buffer[8];

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
        generate_msg(tx_msg, frame_seq_nb, receiver_id, sender_id, 0xE1);
        tx_msg[10] = pos[0];
        tx_msg[11] = pos[1];
        tx_msg[12] = pos[2];

        tx_msg[13] = vel[0];
        tx_msg[14] = vel[1];
        tx_msg[15] = vel[2];

        /////////////////////////////////////////////////////////////////////////////////////////////////////////

        // Store timestamps in the response message.
        resp_msg_set_ts(&tx_msg[RESP_MSG_POLL_RX_TS_IDX + 6], T_reply_end - T_reply_start);

        // Prepare and send the response message
        setTransmitData(sizeof(tx_msg), tx_msg, 1, 1);  // ranging = 1, fcs = 1
        ret = startTransmit(true, true);                // delayed TX, no response expected
        if (ret != DWT_SUCCESS) Serial.println("Failed to schedule delayed TX, too late!");

        // Clear TX frame sent event.
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

        read_position[0] = rx_buffer[10];
        read_position[1] = rx_buffer[11];
        read_position[2] = rx_buffer[12];

        read_velocity[0] = rx_buffer[13];
        read_velocity[1] = rx_buffer[14];
        read_velocity[2] = rx_buffer[15];


        ////////////////////////////////////////////////////////////////////////////////////////////////////////

      } else if (rx_buffer[9] == 0xE2) {  // Check for Final message type
        static double distance = 0;
        uint32_t frame_len;

        uint32_t T_poll_start, T_poll_end, T_reply_start, T_reply_end;
        uint32_t T_round2;
        uint32_t T_reply2;

        float clockOffsetRatio;
        static double tof;
        // Serial.println("Final message received");

        // STEP 1: Compute reply timestamps and clock ratio
        T_poll_start = dwt_readtxtimestamplo32();
        T_poll_end = dwt_readrxtimestamplo32();
        clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

        uint32_t T_reply1, T_round1;
        // STEP 2: Read timestamps from anchor's response (rx_buffer)
        resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &T_reply1);      // T_reply = round2 - reply2
        resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &T_round1);      // T_reply = round2 - reply2
        resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX + 4], &T_reply2);  // T_reply = round2 - reply2

        T_round2 = T_poll_end - T_poll_start;
        ///////////////////////////////////////////////////////

        // STEP 3: Calculate round-trip and distance
        // tof = ((T_round2 - T_reply2 * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
        // distance = tof * SPEED_OF_LIGHT;

        ///////////////////////////////////////////////////////
        // STEP 4: Calculate double sided round-trip distance
        double top_part = ((double)T_round1 * (double)T_round2) - ((double)T_reply1 * (double)T_reply2);
        double bottom_part = (double)T_round1 + (double)T_round2 + (double)T_reply1 + (double)T_reply2;
        tof = top_part / bottom_part;

        distance = tof * SPEED_OF_LIGHT * DWT_TIME_UNITS;

        // Display distance
        snprintf(dist_str, sizeof(dist_str), "DIST: %3.2f m", distance);
        test_run_info((unsigned char *)dist_str);






      } else Serial.println("Ignored msg");
    } else Serial.println("Ignored");
  }

  printRxBuffer(rx_buffer, frame_len);
}

void DW3000_get_robot_info(int8_t position[3], int8_t velocity[3]) {
  // Update values
  Serial.println();
  for (int i = 0; i < 3; i++) {
    position[i] = read_position[i];
    velocity[i] = read_velocity[i];
  }

  Serial.print("Position: [");
  for (int i = 0; i < 3; i++) {
    Serial.print(position[i]);
    if (i < 2) Serial.print(", ");
  }
  Serial.println("]");

  Serial.print("Velocity: [");
  for (int i = 0; i < 3; i++) {
    Serial.print(velocity[i]);
    if (i < 2) Serial.print(", ");
  }
  Serial.println("]");
}




/*****************************************************************************************************************************************************
 * NOTES:
 *
 * Super Deterministic Code (SDC):
 * Since the Ipatov preamble consists of a repeating sequence of the same Ipatov code, the time-of-arrival determined using it is
 * vulnerable to a collision with another packet. If the clock offset between the two packets on the air is low,
 * then the signal from the colliding packet will appear to be just another ray from the desired signal.
 * Depending on when it arrives this can be confused with the first path.
 * The STS uses a continually varying sequence. This means that the colliding packet will not line up with the desired signal.
 * As a result, the TOA will be unaffected.  If security is not a concern, then overhead of key management and counter control
 * is undesirable.  For this reason, the DW3000 includes a special STS mode (SDC) that uses a code optimized for TOA performance.
 * Since it is a time varying sequence optimized for TOA performance it will tolerate packet collisions without requiring any
 * key management.
 * It is important to remember that the SDC mode does not provide security but will increase confidence in the TOA when the
 * on-air packet density is high. For this reason, we would recommend that an SDC based STS is used when security is not a
 * requirement.
 *
 * 1. The sum of the values is the TX to RX antenna delay, experimentally determined by a calibration process. Here we use a hard coded typical value
 *    but, in a real application, each device should have its own antenna delay properly calibrated to get the best possible precision when performing
 *    range measurements.
 * 2. The messages here are similar to those used in the DecaRanging ARM application (shipped with EVK1000 kit). They comply with the IEEE
 *    802.15.4 standard MAC data frame encoding and they are following the ISO/IEC:24730-62:2013 standard. The messages used are:
 *     - a poll message sent by the initiator to trigger the ranging exchange.
 *     - a response message sent by the responder allowing the initiator to go on with the process
 *     - a final message sent by the initiator to complete the exchange and provide all information needed by the responder to compute the
 *       time-of-flight (distance) estimate.
 *    The first 10 bytes of those frame are common and are composed of the following fields:
 *     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: PAN ID (0xDECA).
 *     - byte 5/6: destination address, see NOTE 3 below.
 *     - byte 7/8: source address, see NOTE 3 below.
 *     - byte 9: function code (specific values to indicate which message it is in the ranging process).
 *    The remaining bytes are specific to each message as follows:
 *    Poll message:
 *     - no more data
 *    Response message:
 *     - byte 10: activity code (0x02 to tell the initiator to go on with the ranging exchange).
 *     - byte 11/12: activity parameter, not used for activity code 0x02.
 *    Final message:
 *     - byte 10 -> 13: poll message transmission timestamp.
 *     - byte 14 -> 17: response message reception timestamp.
 *     - byte 18 -> 21: final message transmission timestamp.
 *    All messages end with a 2-byte checksum automatically set by DW3000.
 * 3. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
 *    unique ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
 *    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
 * 4. Delays between frames have been chosen here to ensure proper synchronisation of transmission and reception of the frames between the initiator
 *    and the responder and to ensure a correct accuracy of the computed distance. The user is referred to DecaRanging ARM Source Code Guide for more
 *    details about the timings involved in the ranging process.
 * 5. This timeout is for complete reception of a frame, i.e. timeout duration must take into account the length of the expected frame. Here the value
 *    is arbitrary but chosen large enough to make sure that there is enough time to receive the complete final frame sent by the responder at the
 *    6.81 Mbps data rate used (around 200 us).
 * 6. The preamble timeout allows the receiver to stop listening in situations where preamble is not starting (which might be because the responder is
 *    out of range or did not receive the message to respond to). This saves the power waste of listening for a message that is not coming. We
 *    recommend a minimum preamble timeout of 5 PACs for short range applications and a larger value (e.g. in the range of 50% to 80% of the preamble
 *    length) for more challenging longer range, NLOS or noisy environments.
 * 7. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW3000 OTP memory.
 * 8. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
 *    refer to DW3000 User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
 *    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
 *    bytes.
 * 9. Timestamps and delayed transmission time are both expressed in device time units so we just have to add the desired response delay to poll RX
 *    timestamp to get response transmission time. The delayed transmission time resolution is 512 device time units which means that the lower 9 bits
 *    of the obtained value must be zeroed. This also allows to encode the 40-bit value in a 32-bit words by shifting the all-zero lower 8 bits.
 * 10. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *     automatically appended by the DW3000. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
 *     work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
 * 11. When running this example on the DWK3000 platform with the POLL_RX_TO_RESP_TX_DLY response delay provided, the dwt_starttx() is always
 *     successful. However, in cases where the delay is too short (or something else interrupts the code flow), then the dwt_starttx() might be issued
 *     too late for the configured start time. The code below provides an example of how to handle this condition: In this case it abandons the
 *     ranging exchange and simply goes back to awaiting another poll message. If this error handling code was not here, a late dwt_starttx() would
 *     result in the code flow getting stuck waiting subsequent RX event that will will never come. The companion "initiator" example (ex_05a) should
 *     timeout from awaiting the "response" and proceed to send another poll in due course to initiate another ranging exchange.
 * 12. The high order byte of each 40-bit time-stamps is discarded here. This is acceptable as, on each device, those time-stamps are not separated by
 *     more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays can be handled by a 32-bit
 *     subtraction.
 * 13. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *     DW3000 API Guide for more details on the DW3000 driver functions.
 * 14. In this example, the DW IC is put into IDLE state after calling dwt_initialise(). This means that a fast SPI rate of up to 38 MHz can be used
 *     thereafter.
 * 15. Desired configuration by user may be different to the current programmed configuration. dwt_configure is called to set desired
 *     configuration.
 ****************************************************************************************************************************************************/
