#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
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
#define MAX_FRAME_LEN  (MAX_PAYLOAD_LEN * 2 + MIN_FRAME_LEN + 1)

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

//KOMENDY DLUGOSC PARAMETROW
#define PARAM_LEN_SETINT    5
#define PARAM_LEN_SETGAIN   1
#define PARAM_LEN_SETTIME   1
#define PARAM_LEN_SETLED    1
#define PARAM_LEN_RDARC     5

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
} Command;

//PREFIKSY I ODPOWIEDZ POTWIERDZAJACA
#define RESP_OK             "OK"
#define RESP_ANS_PREFIX     "ANS"
#define GAIN_PREFIX         "GAIN"
#define TIME_PREFIX         "TIME"
#define LED_PREFIX          "LED"
#define INT_PREFIX          "INT"

//KODY BLEDOW TEKSTOWO
#define WRCHSUM_STR "WRCHSUM"
#define WRCMD_STR "WRCMD"
#define WRLEN_STR "WRLEN"
#define WRPOS_STR "WRPOS"
#define WRFRM_STR "WRFRM"
#define WRTIME_STR "WRTIME"
#define NODATA_STR "NODATA"

//KODY BŁEDÓW
typedef enum {
    WRCHSUM,
    WRCMD,
    WRLEN,
    WRPOS,
    WRFRM,
    WRTIME,
    NODATA,
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
    char data[(MAX_PAYLOAD_LEN * 2) + 1];
    Command command;
    char params[MAX_PAYLOAD_LEN + 1];  
    uint8_t params_len;                
    uint16_t crc;
} Frame;


Command parse_command(const char *command_str);
uint8_t get_command_param_len(Command cmd);
ParseResult parse_frame(const char *buffer, size_t len, Frame *frame, char *response_buffer, size_t response_size);
uint16_t calculate_frame_crc(const Frame *frame);
uint8_t build_response_frame(char *buffer, size_t buffer_size, const char *sender,
                         const char *receiver, uint8_t frame_id, const char *response_data, ErrorCode error);
void process_command(Frame *frame, char *response_buffer, size_t response_size);
void process_protocol_data(void);

// GLOBALNE ZMIENNE
extern volatile uint8_t current_gain_index;      // INDEKS GAIN
extern volatile uint8_t current_time_index;      // INDEKS CZASU INTEGRACJI
extern volatile uint8_t led_state;               // STAN LED

// DLUGOSCI TABLIC USTAWIEN
#define GAIN_VALUES_COUNT 4
#define TIME_VALUES_COUNT 5

// TABLICA GAIN
extern const uint8_t GAIN_TABLE[GAIN_VALUES_COUNT];

// TABLICA CZASU INTEGRACJI
extern const uint16_t TIME_TABLE[TIME_VALUES_COUNT];


#endif
