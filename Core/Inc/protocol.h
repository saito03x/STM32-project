#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

//Znaki sterujące
#define PROTOCOL_START_BYTE '&'
#define PROTOCOL_END_BYTE '*'

//Indentyfikator urządzenia
#define DEVICE_ID "STM"

//ROZMIARY PÓL
#define FIELD_START_LEN 1
#define FIELD_ADDR_LEN 3
#define FIELD_DATA_LEN 3
#define FIELD_ID_LEN 2
#define FIELD_CRC_LEN 4
#define FIELD_END_LEN 1

//ROZMIARY DANYCH
#define MAX_PAYLOAD_LEN 256
#define MIN_FRAME_LEN 17

//MAKSYMALNA DLUGOSC RAMKI
#define MAX_FRAME_LEN  (MAX_PAYLOAD_LEN + MIN_FRAME_LEN)

//KOMENDY TEKSTOWO
#define CMD_STR_START   "START"
#define CMD_STR_STOP    "STOP"
#define CMD_STR_SETINT  "SETINT"
#define CMD_STR_SETGAIN "SETGAIN"
#define CMD_STR_SETTIME "SETTIME"
#define CMD_STR_SETLED  "SETLED"
#define CMD_STR_GETINT  "GETINT"
#define CMD_STR_GETGAIN "GETGAIN"
#define CMD_STR_GETTIME "GETTIME"
#define CMD_STR_GETLED  "GETLED"
#define CMD_STR_RDRAW   "RDRAW"
#define CMD_STR_RDARC   "RDARC"
#define CMD_STR_RDHEX   "RDHEX"

//KOMENDY DLUGOSC PARAMETROW
#define PARAM_LEN_SETINT    5
#define PARAM_LEN_SETGAIN   1
#define PARAM_LEN_SETTIME   1
#define PARAM_LEN_SETLED    1
#define PARAM_LEN_RDARC     5
#define PARAM_LEN_RDHEX     5

//KOMENDY ENUM
typedef enum {
    CMD_INVALID = -1,

    START_CMD,
    STOP_CMD,

    SETINT_CMD,
    SETGAIN_CMD,
    SETTIME_CMD,
    SETLED_CMD,

    GETINT_CMD,
    GETGAIN_CMD,
    GETTIME_CMD,
    GETLED_CMD,

    RDRAW_CMD,
    RDARC_CMD,
    RDHEX_CMD,
} Command;

//PREFIKSY I ODPOWIEDZ POTWIERDZAJACA
#define RESP_OK             "OK"
#define RESP_ANS_PREFIX     "ANS"
#define RESP_HEX_PREFIX     "HEX"

//KODY BLEDOW TEKSTOWO
#define WRCHSUM_STR "WRCHSUM"
#define WRCMD_STR "WRCMD"
#define WRLEN_STR "WRLEN"
#define WRPOS_STR "WRPOS"
#define WRFRM_STR "WRFRM"

//KODY BŁEDÓW
typedef enum {
    WRCHSUM,
    WRCMD,
    WRLEN,
    WRPOS,
    WRFRM,
} ErrorCode;

// KODY BŁĘDÓW PARSOWANIA
typedef enum {
    PARSE_OK,              // Parsowanie poprawne
    PARSE_WRONG_RECIPIENT,    // Zły odbiorca (ignoruj ramkę)
    PARSE_TOO_SHORT,          // Ramka za krótka (ignoruj ramkę)
    PARSE_INVALID_FORMAT,     // Błędny format ramki
    PARSE_FORBIDDEN_CHARS,    // Zabronione znaki (& lub *) w ramce
    PARSE_LENGTH_MISMATCH,    // Niezgodność długości danych
    PARSE_CRC_ERROR,          // Błąd CRC
    PARSE_CMD_ERROR           // Nieznana komenda (WRCMD)
} ParseResult;

// STANY MASZYNY DO PARSOWANIA RAMEK
typedef enum {
    STATE_IDLE,
    STATE_HEADER,
    STATE_DATA,
    STATE_CRC_END,
} ParserState;

//STRUKTURA RAMKI

typedef struct {
    char sender[FIELD_ADDR_LEN+1];
    char receiver[FIELD_ADDR_LEN+1];
    uint16_t data_len;
    uint8_t frame_id;
    char data[MAX_PAYLOAD_LEN + 1];
    Command command;
    char params[MAX_PAYLOAD_LEN + 1];  
    uint8_t params_len;                
    uint16_t crc;
} Frame;


int convert_char_to_int(char *str);
uint16_t convert_hex_to_int(const char *hex_str, size_t len);
int hex_decode_string(const char *hex_str, char *output, size_t output_size);
int hex_encode_string(const char *input, char *output, size_t output_size);
Command parse_command(const char *command_str);
uint8_t get_command_param_len(Command cmd);
ParseResult parse_frame(const char *buffer, size_t len, Frame *frame, char *response_buffer, size_t response_size);
uint16_t calculate_frame_crc(const Frame *frame);
bool build_response_frame(char *buffer, size_t buffer_size, const char *sender,
                         const char *receiver, uint8_t frame_id, const char *response_data, ErrorCode error);
void process_protocol_data(void);

// Global configuration variables (extern declarations)
extern volatile uint8_t current_gain_index;      // 0-3 for gain settings
extern volatile uint8_t current_time_index;      // 0-5 for integration time settings
extern volatile uint8_t led_state;               // 0 = OFF, 1 = ON

// Configuration tables (as per protocol specification)
#define GAIN_VALUES_COUNT 4
#define TIME_VALUES_COUNT 6

// Gain settings: 1x, 4x, 16x, 60x
extern const uint8_t GAIN_TABLE[GAIN_VALUES_COUNT];

// Integration time settings: 2.4ms, 24ms, 50ms, 101ms, 154ms, 700ms
extern const uint16_t TIME_TABLE[TIME_VALUES_COUNT];


#endif
