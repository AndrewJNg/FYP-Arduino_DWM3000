
#include "dw3000.h"
#include "SPI.h"

extern SPISettings _fastSPI;

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

#define RNG_DELAY_MS 1000
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

#define ALL_MSG_COMMON_LEN 5
#define ALL_MSG_SN_IDX 2

#define RESP_MSG_POLL_RX_TS_IDX 16
#define RESP_MSG_RESP_TX_TS_IDX 20

#define POLL_RX_TO_RESP_TX_DLY_UUS 450
#define POLL_TX_TO_RESP_RX_DLY_UUS 240
#define RESP_RX_TIMEOUT_UUS 400

static uint8_t frame_seq_nb = 0;
static uint8_t rx_buffer[26];
static uint32_t status_reg = 0;

extern dwt_txconfig_t txconfig_options;

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
  dwt_configuretxrf(&txconfig_options);

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

// void printDistBuffer(uint8_t* buffer, uint16_t length, double distance) {
//   // Decode Bot IDs (assumed positions: sender ID = [5,6], receiver ID = [7,8])
//   char sender_id[3] = { (char)buffer[5], (char)buffer[6], '\0' };
//   char receiver_id[3] = { (char)buffer[7], (char)buffer[8], '\0' };

//   Serial.print("\tSender ID: ");
//   Serial.print(sender_id);
//   Serial.print("\tReceiver ID: ");
//   Serial.print(receiver_id);
//   Serial.print("\tDist: ");
//   Serial.println(distance);

// }