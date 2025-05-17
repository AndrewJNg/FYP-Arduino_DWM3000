#include <SPI.h>
#include <Arduino.h>
#include "dw3000.h"
/* DWM3000 UWB module interface
 Parameters:
 - rstPin: Reset pin
 - irqPin: Interrupt pin
 - ssPin:  Chip select (SPI) pin
 - chip address: Base address
 - debug prints: Enable/disable debug output
*/
class DWM3000 {
public:
  // Constructor
  DWM3000(int rstPin, int irqPin, int ssPin, uint16_t botId, uint16_t Antenna_Delay = 16385, bool debug = false)
    : PIN_RST(rstPin), PIN_IRQ(irqPin), PIN_SS(ssPin), Bot_ID(botId), TX_ANT_DLY(Antenna_Delay), RX_ANT_DLY(Antenna_Delay), dw3000_debug(debug) {
  }

  // Initialize the DWM3000 module
  void begin() {
    UWB_setup();
  }

  // Tag functionality - get distance to another module
  double getDistance(uint16_t receiver_id, float pos[3], float vel[3]) {
    setAnchorMode(false);
    Tag_set_send_mode(Bot_ID, receiver_id, pos, vel);
    return Tag_waiting_for_response(Bot_ID, receiver_id);
  }

  // Anchor functionality - wait for and process incoming messages
  double processIncoming(float pos[3], float vel[3]) {
    setAnchorMode(true);
    return Anchor_waiting_for_response(Bot_ID, pos, vel);
  }

  // Get last received robot info
  // return distance, and update positions and velocity arrays
  uint16_t getRobotInfo(float position[3], float velocity[3]) {
    for (int i = 0; i < 3; i++) {
      position[i] = read_position[i];
      velocity[i] = read_velocity[i];
    }

    if (dw3000_debug) {
      debugPrintRobotInfo();
    }

    return last_receiver_id;
  }

private:
  // Pins
  const int PIN_RST;
  const int PIN_IRQ;
  const int PIN_SS;

  // Configuration
  const uint16_t Bot_ID;
  const uint16_t TX_ANT_DLY;
  const uint16_t RX_ANT_DLY;
  const bool dw3000_debug = true;

  // Constants
  static constexpr uint8_t ALL_MSG_COMMON_LEN = 5;
  static constexpr uint8_t ALL_MSG_SN_IDX = 2;
  static constexpr uint8_t RESP_MSG_POLL_RX_TS_IDX = 10;
  static constexpr uint8_t RESP_MSG_RESP_TX_TS_IDX = 14;
  static constexpr uint16_t POLL_RX_TO_RESP_TX_DLY_UUS = 500;
  static constexpr uint16_t POLL_TX_TO_RESP_RX_DLY_UUS = 240;
  static constexpr uint16_t RESP_RX_TIMEOUT_UUS = 400;

  // State variables
  uint8_t frame_seq_nb = 0;
  uint8_t rx_buffer[34];
  uint32_t status_reg = 0;
  bool anchor = true;
  float read_position[3] = { 0, 0, 0 };
  float read_velocity[3] = { 0, 0, 0 };
  uint16_t last_receiver_id = 0;

  // Configuration structures
  dwt_txconfig_t txconfig_options2 = {
    0x34,       /* PG delay. */
    0xffffffff, /* TX power. */
    0x0         /* PG count */
  };

  dwt_config_t config = {
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

  // Common receive message template
  uint8_t const_receive_msg[7] = { 0x41, 0x88, 0, 0xCA, 0xDE, 0x00, 0x00 };  // Last byte will be replaced with Bot_ID

  // Private methods
  void UWB_setup() {

    // 1. HARD RESET
    pinMode(PIN_RST, OUTPUT);
    digitalWrite(PIN_RST, LOW);
    delay(10);  // hold in reset
    digitalWrite(PIN_RST, HIGH);
    delay(10);  // let chip wake

    // 2. BEGIN SPI
    SPISettings _fastSPI = SPISettings(8000000L, MSBFIRST, SPI_MODE0);
    spiBegin(PIN_IRQ, PIN_RST);
    spiSelect(PIN_SS);
    delay(2);  // short delay

    // 3. SOFTWARE RESET (optional but good)
    dwt_softreset();
    delay(2);  // Time needed for DW3000 to start up

    // Need to make sure DW IC is in IDLE_RC before proceeding
    do {
      if (dw3000_debug) Serial.println("IDLE FAILED");
      dwt_softreset();
      delay(500);  // Extra time given for DW3000 to start up again
    } while (!dwt_checkidlerc() || dwt_initialise(DWT_DW_INIT) == DWT_ERROR);

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
      if (dw3000_debug) Serial.println("INIT FAILED");
      while (1)
        ;
    }

    // Configure GPIOs
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);    //// Optionally Configure GPIOs to show TX/RX activity. ////
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);  //// Optionally enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards. See Note 10 below. /////

    // Configure DW IC
    if (dwt_configure(&config)) {
      if (dw3000_debug) Serial.println("CONFIG FAILED");
      while (1)
        ;
    }

    // Configure the TX spectrum parameters
    dwt_configuretxrf(&txconfig_options2);

    // Set expected response's delay and timeout
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

    // Apply default antenna delay value
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    // Set bot ID in receive message template
    const_receive_msg[6] = Bot_ID & 0xFF;

    if (dw3000_debug) {
      Serial.println("DWM3000 Initialized");
      Serial.println("Setup complete");
    }
  }

  // Set module as tag or anchor
  void setAnchorMode(bool isAnchor) {
    if (isAnchor) {
      if (anchor == false) switchToAnchorMode();
    } else {
      if (anchor == true) switchToTagMode();
    }
  }

  void printRxBuffer(uint8_t *buffer, uint16_t length) {
    if (!dw3000_debug) return;

    buffer[2] = frame_seq_nb;
    Serial.print("Received data: ");
    for (uint16_t i = 0; i < length; i++) {
      if (buffer[i] < 0x10) Serial.print("0");
      Serial.print(buffer[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }

  void generate_msg(uint8_t *tx_msg, uint8_t seq_nb, uint16_t sender_id, uint16_t receiver_id, uint8_t message_type) {
    tx_msg[0] = 0x41;
    tx_msg[1] = 0x88;
    tx_msg[2] = seq_nb;
    tx_msg[3] = 0xCA;
    tx_msg[4] = 0xDE;

    tx_msg[5] = (sender_id >> 8) & 0xFF;
    tx_msg[6] = sender_id & 0xFF;

    tx_msg[7] = (receiver_id >> 8) & 0xFF;
    tx_msg[8] = receiver_id & 0xFF;

    tx_msg[9] = message_type;
  }

  void setTransmitData(int length, uint8_t *buffer, int ranging, int fcs) {
    dwt_writetxdata(length, buffer, 0);
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

  // Tag functionality
  void Tag_set_send_mode(uint16_t sender_id, uint16_t receiver_id, float pos[3], float vel[3]) {
    dwt_forcetrxoff();  // Clean stop any ongoing operations

    // Configure for tag mode
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

    // Clear any pending status bits
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_TXFRS_BIT_MASK);
    delay(10);

    // Prepare message to be sent out
    frame_seq_nb++;
    uint8_t tx_msg[18];
    generate_msg(tx_msg, frame_seq_nb, receiver_id, sender_id, 0xE0);
    // tx_msg[10] = pos[0];
    // tx_msg[11] = pos[1];
    // tx_msg[12] = pos[2];

    // tx_msg[13] = vel[0];
    // tx_msg[14] = vel[1];
    // tx_msg[15] = vel[2];

    // Add compressed pos[] and vel[]
    for (int i = 0; i < 3; i++) {
      int16_t p = (int16_t)(pos[i] * 100);  // e.g., 1.23m → 123
      // int16_t v = (int16_t)(vel[i] * 100);  // e.g., 0.456 m/s → 456

      tx_msg[10 + i * 2] = (p >> 8) & 0xFF;
      tx_msg[11 + i * 2] = p & 0xFF;

      // tx_msg[16 + i * 2] = (v >> 8) & 0xFF;
      // tx_msg[17 + i * 2] = v & 0xFF;
    }

    setTransmitData(sizeof(tx_msg), tx_msg, 1, 1);
    startTransmit(false, true);
  }

  double Tag_waiting_for_response(uint16_t sender_id, uint16_t receiver_id) {
    double distance = 0;

    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {
    };

    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
      distance = Tag_process_received_message(sender_id, receiver_id);
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
      return distance;
    } else {
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
      return -1;
    }
  }

  double Tag_process_received_message(uint16_t sender_id, uint16_t receiver_id) {
    double distance = 0;
    uint32_t frame_len;
    uint32_t T_poll_start, T_poll_end;
    float clockOffsetRatio;

    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_TXFRS_BIT_MASK);

    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (frame_len <= sizeof(rx_buffer)) {
      memset(rx_buffer, 0, sizeof(rx_buffer));
      dwt_readrxdata(rx_buffer, frame_len, 0);
      rx_buffer[ALL_MSG_SN_IDX] = 0;

      if (memcmp(rx_buffer, const_receive_msg, ALL_MSG_COMMON_LEN + 2) == 0) {
        uint32_t resp_tx_time;
        int ret;
        uint64_t T_reply_start, T_reply_end;

        T_reply_start = get_rx_timestamp_u64();
        resp_tx_time = (T_reply_start + (POLL_RX_TO_RESP_TX_DLY_UUS * 4 * UUS_TO_DWT_TIME)) >> 8;
        dwt_setdelayedtrxtime(resp_tx_time);
        T_reply_end = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

        uint8_t tx_msg[24];
        generate_msg(tx_msg, frame_seq_nb, receiver_id, sender_id, 0xE2);

        T_poll_start = dwt_readtxtimestamplo32();
        T_poll_end = dwt_readrxtimestamplo32();
        clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

        uint32_t T_reply1;
        resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX + 6], &T_reply1);

        resp_msg_set_ts(&tx_msg[RESP_MSG_POLL_RX_TS_IDX], T_reply1);
        resp_msg_set_ts(&tx_msg[RESP_MSG_RESP_TX_TS_IDX], T_poll_end - T_poll_start);
        resp_msg_set_ts(&tx_msg[RESP_MSG_RESP_TX_TS_IDX + 4], T_reply_end - T_reply_start);

        uint32_t T_round = T_poll_end - T_poll_start;
        double tof = ((T_round - T_reply1 * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
        distance = tof * SPEED_OF_LIGHT;

        setTransmitData(sizeof(tx_msg), tx_msg, 1, 1);
        ret = startTransmit(true, false);
        if (ret != DWT_SUCCESS && dw3000_debug) {
          Serial.println("Failed to schedule delayed TX, too late!");
        }

        // for (int i = 0; i < 3; i++) {
        //   read_position[i] = rx_buffer[10 + i];
        //   read_velocity[i] = rx_buffer[13 + i];
        // }

        // Obtain pos[] info
        int16_t raw_pos, raw_vel;
        for (int i = 0; i < 3; i++) {
          // Decompress position (stored in bytes 10,11,12,13,14,15)
          raw_pos = (int16_t)((rx_buffer[10 + i * 2] << 8) | rx_buffer[11 + i * 2]);
          read_position[i] = (float)raw_pos / 100.0f;  // cm precision → meters

          // // Decompress velocity (stored in bytes 16,17,18,19,20,21)
          // raw_vel = (int16_t)((rx_buffer[16 + i * 2] << 8) | rx_buffer[17 + i * 2]);
          // read_velocity[i] = (float)raw_vel / 100.0f;  // mm/s precision → m/s
        }


      } else if (dw3000_debug) {
        Serial.println("Ignored");
      }
      printRxBuffer(rx_buffer, frame_len);
    } else {
      distance = -1;
    }

    return distance;
  }

  double Anchor_waiting_for_response(uint16_t sender_id, float pos[3], float vel[3]) {
    const unsigned long timeout_ms = 50;  // 50ms timeout
    unsigned long start_time = millis();
    double distance = -1;

    // Set as Anchor and to activate reception immediately (no timeout).
    dwt_setrxtimeout(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    // Wait for a received frame or error/timeout.
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR))) {
      // if (anchor == 0) break;
      if (millis() - start_time >= timeout_ms) {
        // Serial.println("Anchor RX timeout");
        dwt_forcetrxoff();
        return -1;
      }
    }

    /////////////////////////////////////////////////////////

    // If a valid frame is received, process the message.
    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
      distance = Anchor_process_received_message(sender_id, pos, vel);

      // Clear good RX frame event in the DW IC status register.
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
    } else {
      // Clear RX error events in the DW IC status register.
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
      distance = -1;
    }
    return distance;
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  double Anchor_process_received_message(uint16_t sender_id, float pos[3], float vel[3]) {
    uint32_t frame_len;
    double distance = -1;

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
          // tx_msg[10] = pos[0];
          // tx_msg[11] = pos[1];
          // tx_msg[12] = pos[2];

          // tx_msg[13] = vel[0];
          // tx_msg[14] = vel[1];
          // tx_msg[15] = vel[2];
          // Add compressed pos[] and vel[]
          for (int i = 0; i < 3; i++) {
            int16_t p = (int16_t)(pos[i] * 100);  // e.g., 1.23m → 123
            // int16_t v = (int16_t)(vel[i] * 100);  // e.g., 0.456 m/s → 456

            tx_msg[10 + i * 2] = (p >> 8) & 0xFF;
            tx_msg[11 + i * 2] = p & 0xFF;

            // tx_msg[16 + i * 2] = (v >> 8) & 0xFF;
            // tx_msg[17 + i * 2] = v & 0xFF;
          }

          /////////////////////////////////////////////////////////////////////////////////////////////////////////

          // Store timestamps in the response message.
          resp_msg_set_ts(&tx_msg[RESP_MSG_POLL_RX_TS_IDX + 6], T_reply_end - T_reply_start);

          // Prepare and send the response message
          setTransmitData(sizeof(tx_msg), tx_msg, 1, 1);  // ranging = 1, fcs = 1
          ret = startTransmit(true, true);                // delayed TX, no response expected
          if (ret != DWT_SUCCESS) Serial.println("Failed to schedule delayed TX, too late!");

          // Clear TX frame sent event.
          dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

          // read_position[0] = rx_buffer[10];
          // read_position[1] = rx_buffer[11];
          // read_position[2] = rx_buffer[12];

          // read_velocity[0] = rx_buffer[13];
          // read_velocity[1] = rx_buffer[14];
          // read_velocity[2] = rx_buffer[15];

          // Obtain pos[] info
          int16_t raw_pos, raw_vel;
          for (int i = 0; i < 3; i++) {
            // Decompress position (stored in bytes 10,11,12,13,14,15)
            raw_pos = (int16_t)((rx_buffer[10 + i * 2] << 8) | rx_buffer[11 + i * 2]);
            read_position[i] = (float)raw_pos / 100.0f;  // cm precision → meters

            // // Decompress velocity (stored in bytes 16,17,18,19,20,21)
            // raw_vel = (int16_t)((rx_buffer[16 + i * 2] << 8) | rx_buffer[17 + i * 2]);
            // read_velocity[i] = (float)raw_vel / 100.0f;  // mm/s precision → m/s
          }
          last_receiver_id = receiver_id;


          ////////////////////////////////////////////////////////////////////////////////////////////////////////

        } else if (rx_buffer[9] == 0xE2) {  // Check for Final message type
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
          // snprintf(dist_str, sizeof(dist_str), "DIST: %3.2f m", distance);
          // test_run_info((unsigned char *)dist_str);
          last_receiver_id = receiver_id;






        } else Serial.println("Ignored msg");
      } else {

        if (rx_buffer[9] == 0xE0) {  // if msg not for me, just read address and position
          // Serial.println("new msg");
          // read_position[0] = rx_buffer[10];
          // read_position[1] = rx_buffer[11];
          // read_position[2] = rx_buffer[12];

          // read_velocity[0] = rx_buffer[13];
          // read_velocity[1] = rx_buffer[14];
          // read_velocity[2] = rx_buffer[15];

          // Obtain pos[] info
          int16_t raw_pos, raw_vel;
          for (int i = 0; i < 3; i++) {
            // Decompress position (stored in bytes 10,11,12,13,14,15)
            raw_pos = (int16_t)((rx_buffer[10 + i * 2] << 8) | rx_buffer[11 + i * 2]);
            read_position[i] = (float)raw_pos / 100.0f;  // cm precision → meters

            // // Decompress velocity (stored in bytes 16,17,18,19,20,21)
            // raw_vel = (int16_t)((rx_buffer[16 + i * 2] << 8) | rx_buffer[17 + i * 2]);
            // read_velocity[i] = (float)raw_vel / 100.0f;  // mm/s precision → m/s
          }
          last_receiver_id = receiver_id;
          distance = -2;
        }
      }
    }

    printRxBuffer(rx_buffer, frame_len);

    return distance;
  }

  void switchToTagMode() {
    dwt_forcetrxoff();
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_TXFRS_BIT_MASK);
    anchor = false;
    if (dw3000_debug) Serial.println("Switched to TAG mode");
  }

  void switchToAnchorMode() {
    dwt_forcetrxoff();
    dwt_setrxtimeout(0);
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_TXFRS_BIT_MASK);
    anchor = true;
    if (dw3000_debug) Serial.println("Switched to ANCHOR mode");
  }

  void debugPrintRobotInfo() {
    Serial.println();
    Serial.print("Position: [");
    for (int i = 0; i < 3; i++) {
      Serial.print(read_position[i]);
      if (i < 2) Serial.print(", ");
    }
    Serial.println("]");

    Serial.print("Velocity: [");
    for (int i = 0; i < 3; i++) {
      Serial.print(read_velocity[i]);
      if (i < 2) Serial.print(", ");
    }
    Serial.println("]");
  }
};