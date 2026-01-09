#include "protocol.h"
#include "crc16.h"
#include "circular_buffer.h"
#include "gpio.h"
#include "tim.h"
#include "i2c.h"
#include "tcs34725.h"
#include <string.h>
#include <stdio.h>

volatile uint8_t current_gain_index = 0;      // Default: 1x gain
volatile uint8_t current_time_index = 3;      // Default: 154ms integration
volatile uint8_t led_state = 0;               // Default: LED OFF

extern volatile uint32_t timer_interval;


const uint8_t GAIN_TABLE[GAIN_VALUES_COUNT] = {
    TCS34725_GAIN_1X,
    TCS34725_GAIN_4X,
    TCS34725_GAIN_16X,
    TCS34725_GAIN_60X
};

const uint16_t TIME_TABLE[TIME_VALUES_COUNT] = {
    TCS34725_INTEGRATIONTIME_2_4MS,
    TCS34725_INTEGRATIONTIME_24MS,
    TCS34725_INTEGRATIONTIME_101MS,
    TCS34725_INTEGRATIONTIME_154MS,
    TCS34725_INTEGRATIONTIME_700MS,
};



static uint8_t is_valid_sender(const char *sender) {
	if (strlen(sender) != FIELD_ADDR_LEN) {
		return 0;
	}
	for (int i = 0; i < FIELD_ADDR_LEN; i++) {
		if (sender[i] == PROTOCOL_START_BYTE || sender[i] == PROTOCOL_END_BYTE) {
			return 0;
		}
	}
	return 1;
}

static int format_ans_data(char *buffer, size_t buffer_size,
		TCS34725_Data_t *data) {
	return snprintf(buffer, buffer_size, "ANS"
			"R%05u"
			"G%05u"
			"B%05u"
			"C%05u", data->r, data->g, data->b, data->c);
}


static int convert_char_to_int(char *str) {
	int result = 0;
	int len = strlen(str);
	for (int i = 0; i < len; i++) {
		if (str[i] < '0' || str[i] > '9') {
			return -1; // Nie cyfra
		}
		result = result * 10 + (str[i] - '0');
	}
	return result;
}


static uint16_t convert_hex_to_int(const char *hex_str, size_t len) {
	uint16_t result = 0;
	for (size_t i = 0; i < len; i++) {
		result <<= 4;
		if (hex_str[i] >= '0' && hex_str[i] <= '9') {
			result += hex_str[i] - '0';
		} else if (hex_str[i] >= 'A' && hex_str[i] <= 'F') {
			result += hex_str[i] - 'A' + 10;
		} else {
			return 0; // Bląd konwersji hex
		}
	}
	return result;
}


static uint16_t get_integration_time_ms(uint8_t index) {
    switch (index) {
        case 0: return 3;   // 2.4ms
        case 1: return 24;  // 24ms
        case 2: return 101; // 101ms
        case 3: return 154; // 154ms
        case 4: return 700; // 700ms
        default: return 0;
    }
}

static int hex_decode_string(const char *hex_str, char *output, size_t output_size) {
	if (!hex_str || !output || output_size == 0) {
		return -1;
	}

	size_t hex_len = strlen(hex_str);
	if (hex_len % 2 != 0) {
		return -1;
	}

	size_t byte_count = hex_len / 2;
	if (byte_count >= output_size) {
		return -1;
	}

	for (size_t i = 0; i < byte_count; i++) {
		char hex_pair[3] = { hex_str[i * 2], hex_str[i * 2 + 1], '\0' };
		uint16_t byte_val = convert_hex_to_int(hex_pair, 2);
		if (byte_val == 0 && strcmp(hex_pair, "00") != 0) {
			return -1;
		}
		output[i] = (char) byte_val;
	}

	output[byte_count] = '\0';
	return (int) byte_count;
}


static int hex_encode_string(const char *input, char *output, size_t output_size) {
	if (!input || !output || output_size == 0) {
		return -1;
	}

	size_t input_len = strlen(input);
	size_t required_size = input_len * 2 + 1;

	if (required_size > output_size) {
		return -1;
	}

	for (size_t i = 0; i < input_len; i++) {
		sprintf(&output[i * 2], "%02X", (uint8_t) input[i]);
	}

	output[input_len * 2] = '\0';
	return (int) (input_len * 2);
}


Command parse_command(const char *command_str) {
	if (!command_str) {
		return CMD_INVALID;
	}

	if (strcmp(command_str, CMD_STR_START) == 0) {
		return START_CMD;
	}
	if (strcmp(command_str, CMD_STR_STOP) == 0) {
		return STOP_CMD;
	}
	if (strcmp(command_str, CMD_STR_SETINT) == 0) {
		return SETINT_CMD;
	}
	if (strcmp(command_str, CMD_STR_SETGAIN) == 0) {
		return SETGAIN_CMD;
	}
	if (strcmp(command_str, CMD_STR_SETTIME) == 0) {
		return SETTIME_CMD;
	}
	if (strcmp(command_str, CMD_STR_GETINT) == 0) {
		return GETINT_CMD;
	}
	if (strcmp(command_str, CMD_STR_GETGAIN) == 0) {
		return GETGAIN_CMD;
	}
	if (strcmp(command_str, CMD_STR_GETTIME) == 0) {
		return GETTIME_CMD;
	}
	if (strcmp(command_str, CMD_STR_RDRAW) == 0) {
		return RDRAW_CMD;
	}
	if (strcmp(command_str, CMD_STR_RDARC) == 0) {
		return RDARC_CMD;
	}
	if (strcmp(command_str, CMD_STR_SETLED) == 0) {
		return SETLED_CMD;
	}
	if (strcmp(command_str, CMD_STR_GETLED) == 0) {
		return GETLED_CMD;
	}

	return CMD_INVALID;
}


uint8_t get_command_param_len(Command cmd) {
	switch (cmd) {
	case SETINT_CMD:
		return PARAM_LEN_SETINT;
	case SETGAIN_CMD:
		return PARAM_LEN_SETGAIN;
	case SETTIME_CMD:
		return PARAM_LEN_SETTIME;
	case RDARC_CMD:
		return PARAM_LEN_RDARC;
	case SETLED_CMD:
		return PARAM_LEN_SETLED;
	case START_CMD:
	case STOP_CMD:
	case GETINT_CMD:
	case GETGAIN_CMD:
	case GETTIME_CMD:
	case GETLED_CMD:
	case RDRAW_CMD:
	default:
		return 0;
	}
}


ParseResult parse_frame(const char *buffer, size_t len, Frame *frame,
		char *response_buffer, size_t response_size) {

	if (!buffer || !frame) {
		return PARSE_INVALID_FORMAT;
	}

	// Sprawdź minimalną długość ramki
	if (len < MIN_FRAME_LEN) {
		return PARSE_TOO_SHORT;
	}

	// Znalezienie poczatku ramki
	size_t start_pos = 0;
	for (size_t i = 0; i < len; i++) {
		if (buffer[i] == PROTOCOL_START_BYTE) {
			start_pos = i;
			break;
		}
	}

	if (start_pos + MIN_FRAME_LEN > len) {
		return PARSE_TOO_SHORT;
	}

	// Sprawdzenie czy jestesmy na koncu ramki
	if (buffer[len - 1] != PROTOCOL_END_BYTE) {
		return PARSE_INVALID_FORMAT;
	}

	size_t pos = start_pos + 1; // Po znaku startowym

	// Sprawdza zabornione znaki & i * tylko w nagłowku
	size_t header_end = start_pos + 1 + FIELD_ADDR_LEN + FIELD_ADDR_LEN
			+ FIELD_DATA_LEN + FIELD_ID_LEN;
	for (size_t i = start_pos + 1; i < header_end && i < len; i++) {
		if (buffer[i] == PROTOCOL_START_BYTE || buffer[i] == PROTOCOL_END_BYTE) {
			return PARSE_FORBIDDEN_CHARS;
		}
	}

	//Sprwdzanie czy długosc ramki jest poprawna
	if (pos + FIELD_ADDR_LEN > len) {
		return PARSE_INVALID_FORMAT;
	}

	memcpy(frame->sender, &buffer[pos], FIELD_ADDR_LEN);
	frame->sender[FIELD_ADDR_LEN] = '\0';
	pos += FIELD_ADDR_LEN;

	//Sprwdzanie czy długosc ramki jest poprawna
	if (pos + FIELD_ADDR_LEN > len) {
		return PARSE_INVALID_FORMAT;
	}

	memcpy(frame->receiver, &buffer[pos], FIELD_ADDR_LEN);
	frame->receiver[FIELD_ADDR_LEN] = '\0';
	pos += FIELD_ADDR_LEN;

	// Sprawdza czy odbiorca to STM
	if (strcmp(frame->receiver, DEVICE_ID) != 0) {
		return PARSE_WRONG_RECIPIENT;
	}

	//Sprwdzanie czy długość ramki jest poprawna
	if (pos + FIELD_DATA_LEN > len) {
		return PARSE_INVALID_FORMAT;
	}

	char len_str[FIELD_DATA_LEN + 1];
	memcpy(len_str, &buffer[pos], FIELD_DATA_LEN);
	len_str[FIELD_DATA_LEN] = '\0';
	frame->data_len = convert_char_to_int(len_str);
	if (frame->data_len < 0 || frame->data_len > MAX_PAYLOAD_LEN) {
		return PARSE_INVALID_FORMAT;
	}
	pos += FIELD_DATA_LEN;

	//Sprwdzanie czy długość ramki jest poprawna
	if (pos + FIELD_ID_LEN > len) {
		return PARSE_INVALID_FORMAT;
	}

	char id_str[FIELD_ID_LEN + 1];
	memcpy(id_str, &buffer[pos], FIELD_ID_LEN);
	id_str[FIELD_ID_LEN] = '\0';
	frame->frame_id = convert_char_to_int(id_str);
	if(frame->frame_id < 0) return PARSE_INVALID_FORMAT;
	pos += FIELD_ID_LEN;

	//Sprwdzanie czy długość ramki jest poprawna
	if (pos + frame->data_len > len) {
		return PARSE_INVALID_FORMAT;
	}

	if (frame->data_len > 0) {
		char hex_data[(MAX_PAYLOAD_LEN * 2) + 1];
		memcpy(hex_data, &buffer[pos], frame->data_len);
		hex_data[frame->data_len] = '\0';
		if (frame->data_len % 2 != 0) {
			return PARSE_LENGTH_MISMATCH;
		}

		//Dekodowanie napisu w HEX ASCII na zanki ASCII
		int decoded_len = hex_decode_string(hex_data, frame->data,
				sizeof(frame->data));
		if (decoded_len < 0) {
			return PARSE_INVALID_FORMAT;
		}
		frame->data_len = decoded_len;
	}

	pos += (frame->data_len * 2);

	if (pos + FIELD_CRC_LEN > len - 1) {
		return PARSE_INVALID_FORMAT;
	}

	char crc_str[FIELD_CRC_LEN + 1];
	memcpy(crc_str, &buffer[pos], FIELD_CRC_LEN);
	crc_str[FIELD_CRC_LEN] = '\0';

	//Sprawdza zabornione znaki w crc
	for (size_t i = pos; i < pos + FIELD_CRC_LEN; i++) {
		if (buffer[i] == PROTOCOL_START_BYTE || buffer[i] == PROTOCOL_END_BYTE) {
			return PARSE_FORBIDDEN_CHARS;
		}
	}

	frame->crc = convert_hex_to_int(crc_str, FIELD_CRC_LEN);
	if (frame->crc == 0 && strcmp(crc_str, "0000") != 0) {
		return PARSE_INVALID_FORMAT;
	}
	pos += FIELD_CRC_LEN;

	//Sprawdzenie czy jestesmy na koncu ramki
	if (pos != len - 1) {
		return PARSE_INVALID_FORMAT;
	}

	//Obliczenie crc z odebranych danych i porownanie
	uint16_t calculated_crc = calculate_frame_crc(frame);

	if (calculated_crc != frame->crc) {
		if (response_buffer && response_size >= MAX_PAYLOAD_LEN
				&& strlen(frame->sender) == FIELD_ADDR_LEN) {
			if (is_valid_sender(frame->sender)) {
				build_response_frame(response_buffer, response_size, DEVICE_ID,
						frame->sender, frame->frame_id, NULL, WRCHSUM);
			}
		}
		return PARSE_CRC_ERROR;
	}

	// Znajdywanie długości nazwy komendy
	size_t cmd_name_len = 0;
	for (size_t i = 0; i < frame->data_len && i < MAX_PAYLOAD_LEN; i++) {
		if (frame->data[i] >= 'A' && frame->data[i] <= 'Z') {
			cmd_name_len++;
		} else {
			break; // Koniec nazwy komendy
		}
	}

	if (cmd_name_len == 0) {
		// Brak nazwy komendy
		if (response_buffer && response_size >= MAX_PAYLOAD_LEN
				&& is_valid_sender(frame->sender)) {
			build_response_frame(response_buffer, response_size, DEVICE_ID,
					frame->sender, frame->frame_id, NULL, WRCMD);
		}
		return PARSE_CMD_ERROR;
	}

	//Wyodrębnienie nazwy komendy
	char cmd_name[32];
	//Sprawdza czy długość nazwy komendy jest poprawna, jezeli nie to skraca do maksymalnej długości, aby moc zapisac do stringa pusty znak na koncu
	if (cmd_name_len >= sizeof(cmd_name)) {
		cmd_name_len = sizeof(cmd_name) - 1;
	}
	memcpy(cmd_name, frame->data, cmd_name_len);
	cmd_name[cmd_name_len] = '\0';

	//Parsowanie komendy
	Command cmd = parse_command(cmd_name);
	if (cmd == CMD_INVALID) {
		//Nieznana komenda
		if (response_buffer && response_size >= MAX_PAYLOAD_LEN
				&& is_valid_sender(frame->sender)) {
			build_response_frame(response_buffer, response_size, DEVICE_ID,
					frame->sender, frame->frame_id, NULL, WRCMD);
		}
		return PARSE_CMD_ERROR;
	}

	uint8_t expected_param_len = get_command_param_len(cmd);

	if (expected_param_len == 0) {
		if (frame->data_len != cmd_name_len) {
			if (response_buffer && response_size >= MAX_PAYLOAD_LEN
					&& is_valid_sender(frame->sender)) {
				build_response_frame(response_buffer, response_size,
				DEVICE_ID, frame->sender, frame->frame_id, NULL, WRCMD);
			}
			return PARSE_CMD_ERROR;
		}
	} else {
		size_t expected_total_len = cmd_name_len + expected_param_len;
		if (frame->data_len != expected_total_len) {
			if (response_buffer && response_size >= MAX_PAYLOAD_LEN
					&& is_valid_sender(frame->sender)) {
				build_response_frame(response_buffer, response_size,
				DEVICE_ID, frame->sender, frame->frame_id, NULL, WRLEN);
			}
			return PARSE_LENGTH_MISMATCH;
		}
	}

	frame->command = cmd;

	frame->params_len = 0;
	frame->params[0] = '\0';
	if (expected_param_len > 0) {
		if (cmd_name_len < frame->data_len) {
			size_t param_start = cmd_name_len;
			size_t param_len = frame->data_len - cmd_name_len;
			if (param_len > MAX_PAYLOAD_LEN) {
				param_len = MAX_PAYLOAD_LEN;
			}
			memcpy(frame->params, &frame->data[param_start], param_len);
			frame->params[param_len] = '\0';
			frame->params_len = param_len;
		}
	}
	return PARSE_OK;
}


uint16_t calculate_frame_crc(const Frame *frame) {
	if (!frame)
		return 0;

	char crc_buffer[MAX_FRAME_LEN];
	size_t pos = 0;

	memcpy(&crc_buffer[pos], frame->sender, FIELD_ADDR_LEN);
	pos += FIELD_ADDR_LEN;

	memcpy(&crc_buffer[pos], frame->receiver, FIELD_ADDR_LEN);
	pos += FIELD_ADDR_LEN;

	char len_str[4];
	int hex_data_len = frame->data_len * 2;
	sprintf(len_str, "%03d", hex_data_len);
	memcpy(&crc_buffer[pos], len_str, FIELD_DATA_LEN);
	pos += FIELD_DATA_LEN;

	char id_str[3];
	sprintf(id_str, "%02d", frame->frame_id);
	memcpy(&crc_buffer[pos], id_str, FIELD_ID_LEN);
	pos += FIELD_ID_LEN;

	if (frame->data_len > 0) {
		char hex_data[MAX_PAYLOAD_LEN * 2 + 1];
		int hex_len = hex_encode_string(frame->data, hex_data,
				sizeof(hex_data));
		if (hex_len > 0) {
			memcpy(&crc_buffer[pos], hex_data, hex_len);
			pos += hex_len;
		}
	}

	return crc16_ccitt((const uint8_t*) crc_buffer, pos);
}

uint8_t build_response_frame(char *buffer, size_t buffer_size, const char *sender,
		const char *receiver, uint8_t frame_id, const char *response_data,
		ErrorCode error) {
	if (!buffer || !sender || buffer_size < MIN_FRAME_LEN) {
		return 0;
	}

	char raw_data[MAX_PAYLOAD_LEN];

	if (response_data != NULL) {
		strncpy(raw_data, response_data, sizeof(raw_data) - 1);
		raw_data[sizeof(raw_data) - 1] = '\0';
	} else {
		switch (error) {
		case WRCHSUM:
			strncpy(raw_data, WRCHSUM_STR, sizeof(raw_data) - 1);
			break;
		case WRCMD:
			strncpy(raw_data, WRCMD_STR, sizeof(raw_data) - 1);
			break;
		case WRLEN:
			strncpy(raw_data, WRLEN_STR, sizeof(raw_data) - 1);
			break;
		case WRPOS:
			strncpy(raw_data, WRPOS_STR, sizeof(raw_data) - 1);
			break;
		case WRFRM:
			strncpy(raw_data, WRFRM_STR, sizeof(raw_data) - 1);
			break;
		case WRTIME:
			strncpy(raw_data, WRTIME_STR, sizeof(raw_data) - 1);
			break;
		default:
			strncpy(raw_data, WRFRM_STR, sizeof(raw_data) - 1);
			break;
		}
		raw_data[sizeof(raw_data) - 1] = '\0';
	}

	char hex_data[MAX_PAYLOAD_LEN * 2 + 1];
	int hex_len = hex_encode_string(raw_data, hex_data, sizeof(hex_data));
	if (hex_len < 0) {
		return 0;
	}

	const char *data = hex_data;
	size_t data_len = hex_len;

	size_t total_len = FIELD_START_LEN + FIELD_ADDR_LEN + FIELD_ADDR_LEN
			+ FIELD_DATA_LEN +
			FIELD_ID_LEN + data_len + FIELD_CRC_LEN + FIELD_END_LEN;
	if (total_len > buffer_size) {
		return 0;
	}

	size_t pos = 0;

	buffer[pos++] = PROTOCOL_START_BYTE;

	memcpy(&buffer[pos], sender, FIELD_ADDR_LEN);
	pos += FIELD_ADDR_LEN;

	memcpy(&buffer[pos], receiver, FIELD_ADDR_LEN);
	pos += FIELD_ADDR_LEN;

	char len_str[4];
	sprintf(len_str, "%03d", data_len);
	memcpy(&buffer[pos], len_str, FIELD_DATA_LEN);
	pos += FIELD_DATA_LEN;

	char id_str[3];
	sprintf(id_str, "%02d", frame_id);
	memcpy(&buffer[pos], id_str, FIELD_ID_LEN);
	pos += FIELD_ID_LEN;

	memcpy(&buffer[pos], data, data_len);
	pos += data_len;

	//Bez poczatku i konca ramki
	char crc_buffer[MAX_FRAME_LEN - 2];
	size_t crc_pos = 0;

	memcpy(&crc_buffer[crc_pos], sender, FIELD_ADDR_LEN);
	crc_pos += FIELD_ADDR_LEN;
	memcpy(&crc_buffer[crc_pos], receiver, FIELD_ADDR_LEN);
	crc_pos += FIELD_ADDR_LEN;
	memcpy(&crc_buffer[crc_pos], len_str, FIELD_DATA_LEN);
	crc_pos += FIELD_DATA_LEN;
	memcpy(&crc_buffer[crc_pos], id_str, FIELD_ID_LEN);
	crc_pos += FIELD_ID_LEN;
	memcpy(&crc_buffer[crc_pos], data, data_len);
	crc_pos += data_len;

	uint16_t crc = crc16_ccitt((const uint8_t*) crc_buffer, crc_pos);

	sprintf(&buffer[pos], "%04X", crc);
	pos += FIELD_CRC_LEN;

	buffer[pos++] = PROTOCOL_END_BYTE;

	buffer[pos] = '\0';

	return 1;
}


void process_received_frame(const char *buffer, uint16_t len) {
	Frame frame;
	char response[MAX_FRAME_LEN];
	ParseResult result = parse_frame(buffer, len, &frame, response,
			sizeof(response));
	if (result == PARSE_OK) {
		process_command(&frame, response, sizeof(response));
	} else if (result == PARSE_CRC_ERROR) {
		if (strlen(response) > 0) {
			UART_TX_FSend("%s", response);
		}
	} else if (result == PARSE_LENGTH_MISMATCH) {
		if (is_valid_sender(frame.sender)) {
			if (build_response_frame(response, sizeof(response), DEVICE_ID,
					frame.sender, frame.frame_id, NULL, WRLEN)) {
				UART_TX_FSend("%s", response);
			}
		}
	} else if (result == PARSE_CMD_ERROR) {
		if (strlen(response) > 0) {
			UART_TX_FSend("%s", response);
		}
	} else if (result == PARSE_INVALID_FORMAT) {
		if (is_valid_sender(frame.sender)) {
			if (build_response_frame(response, sizeof(response), DEVICE_ID,
					frame.sender, 0, NULL, WRFRM)) {
				UART_TX_FSend("%s", response);
			}
		}
	}
	//PARSE_TOO_SHORT, PARSE_WRONG_RECIPIENT, PARSE_FORBIDDEN_CHARS ingorowanie bez odpowiedzi
}


void process_protocol_data(void) {
	static char frame_buffer[MAX_FRAME_LEN];
	static size_t buffer_pos = 0;
	static ParserState state = STATE_IDLE;
	static uint16_t expected_data_len = 0;  // Długość danych z pola Len
	static size_t header_pos = 0;          // Pozycja w nagłówku
	static size_t data_pos = 0;            // Pozycja w danych
	static size_t crc_pos = 0;             // Pozycja w CRC

	// Przetwarza wszystkie dostępne znaki z bufora
	while (!UART_RX_IsEmpty()) {
		int16_t received_char = UART_RX_GetChar();
		if (received_char == -1) {
			break; // Brak danych
		}

		char c = (char) received_char;

		switch (state) {
		case STATE_IDLE:
			// Czekanie na znak startu &
			if (c == PROTOCOL_START_BYTE) {
				buffer_pos = 0;
				frame_buffer[buffer_pos++] = c;
				header_pos = 0;
				state = STATE_HEADER;
			}
			// Ignoruje wszystko przed &
			break;

		case STATE_HEADER:
			// Sprawdza czy to nowy znak startu, jezeli tak to reset parsera
			if (c == PROTOCOL_START_BYTE) {
				buffer_pos = 0;
				frame_buffer[buffer_pos++] = c;
				header_pos = 0;
				break;
			}

			// Sprawdza zabronione znaki w nagłówku
			if (c == PROTOCOL_END_BYTE) {
				state = STATE_IDLE;
				buffer_pos = 0;
				header_pos = 0;
				break;
			}

			// Pobieranie nagłówka Sender(3) + Receiver(3) + Len(3) + ID(2) = 11
			frame_buffer[buffer_pos++] = c;
			header_pos++;

			// Sprawdza czy odebrano cały nagłówek
			if (header_pos
					>= FIELD_ADDR_LEN + FIELD_ADDR_LEN + FIELD_DATA_LEN
							+ FIELD_ID_LEN) {

				// Parsuj długość danych z nagłówka
				char len_str[FIELD_DATA_LEN + 1];
				// Kopiowanie danych z bufora do len_str
				memcpy(len_str,
						&frame_buffer[1 + FIELD_ADDR_LEN + FIELD_ADDR_LEN],
						FIELD_DATA_LEN);
				//Dodanie znaku pustego na koniec stringa
				len_str[FIELD_DATA_LEN] = '\0';

				expected_data_len = convert_char_to_int(len_str);

				// Sprawdza poprawność długości
				if (expected_data_len < 0 || expected_data_len > MAX_PAYLOAD_LEN) {
					// Błędna długość i reset
					state = STATE_IDLE;
					buffer_pos = 0;
					break;
				}

				// Pobieranie danych
				data_pos = 0;
				state = STATE_DATA;
			}
			break;

		case STATE_DATA:
			if (c == PROTOCOL_START_BYTE) {
				buffer_pos = 0;
				frame_buffer[buffer_pos++] = c;
				header_pos = 0;
				data_pos = 0;
				expected_data_len = 0;
				crc_pos = 0;
				state = STATE_HEADER;
				break;
			}

			frame_buffer[buffer_pos++] = c;
			data_pos++;

			//Sprawdza czy maksymalny rozmiar nie jest przekroczony
			if (buffer_pos >= MAX_FRAME_LEN) {
				state = STATE_IDLE;
				buffer_pos = 0;
				break;
			}

			// Sprawdza czy odebrano wszystkie dane
			if (data_pos >= expected_data_len) {
				// Przejscie do pobierania CRC
				crc_pos = 0;
				state = STATE_CRC_END;
			}
			break;

		case STATE_CRC_END:
			// Sprawdza czy to nowy znak startu
			if (c == PROTOCOL_START_BYTE) {
				buffer_pos = 0;
				frame_buffer[buffer_pos++] = c;
				header_pos = 0;
				crc_pos = 0;
				state = STATE_HEADER;
				break;
			}

			// Sprawdza czy przyszedł znak końca ramki
			if (c == PROTOCOL_END_BYTE) {
				frame_buffer[buffer_pos++] = c;
				process_received_frame(frame_buffer, buffer_pos);
				state = STATE_IDLE;
				buffer_pos = 0;
				crc_pos = 0;
				break;
			}

			// Jeśli to nie gwiazdka, zbiera znaki CRC
			frame_buffer[buffer_pos++] = c;
			crc_pos++;

			// Sprawdza czy długosc crc jest poprawna
			if (crc_pos > FIELD_CRC_LEN) {
				state = STATE_IDLE;
				buffer_pos = 0;
				crc_pos = 0;
			}

			// Sprawdza czy bufor nie jest przepelniony
			if (buffer_pos >= MAX_FRAME_LEN) {
				state = STATE_IDLE;
				buffer_pos = 0;
				crc_pos = 0;
			}
			break;
		}
	}
}


void process_command(Frame *frame, char *response_buffer, size_t response_size) {
	char data_buffer[MAX_PAYLOAD_LEN];
	ErrorCode error = 0;

	switch (frame->command) {
	case START_CMD:
		if (build_response_frame(response_buffer, response_size, DEVICE_ID,
				frame->sender, frame->frame_id, RESP_OK, 0)) {
			HAL_TIM_Base_Start_IT(&htim3);
			UART_TX_FSend("%s", response_buffer);
		}
		break;

	case STOP_CMD:
		if (build_response_frame(response_buffer, response_size, DEVICE_ID,
				frame->sender, frame->frame_id, RESP_OK, 0)) {
			HAL_TIM_Base_Stop_IT(&htim3);
			UART_TX_FSend("%s", response_buffer);
		}
		break;

	case RDRAW_CMD:
	{
		ColorBufferEntry_t *latest = ColorBuffer_GetLatest();
		if (latest != NULL) {
			format_ans_data(data_buffer, sizeof(data_buffer), &latest->data);
			if (build_response_frame(response_buffer, response_size, DEVICE_ID,
					frame->sender, frame->frame_id, data_buffer, 0)) {
				UART_TX_FSend("%s", response_buffer);
			}
		} else {
			sprintf(data_buffer, NODATA_STR);
			if (build_response_frame(response_buffer, response_size, DEVICE_ID,
					frame->sender, frame->frame_id, data_buffer, 0)) {
				UART_TX_FSend("%s", response_buffer);
			}
		}
	}
		break;

	case RDARC_CMD:
	{
		if (frame->params_len != PARAM_LEN_RDARC) {
			error = WRLEN;
		} else {
			int time_offset = convert_char_to_int(frame->params);
			if (time_offset < 0) {
				error = WRCMD;
			} else {
				uint32_t max_offset = COLOR_BUFFER_SIZE * timer_interval;
				if (time_offset == 0 || time_offset > max_offset) {
					error = WRPOS;
				} else {
					ColorBufferEntry_t *entry = ColorBuffer_GetByTimeOffset(
							time_offset);
					if (entry != NULL) {
						format_ans_data(data_buffer, sizeof(data_buffer),
								&entry->data);
						if (build_response_frame(response_buffer, response_size,
						DEVICE_ID, frame->sender, frame->frame_id, data_buffer,
								0)) {
							UART_TX_FSend("%s", response_buffer);
						}
					} else {
						sprintf(data_buffer, NODATA_STR);
						if (build_response_frame(response_buffer, response_size,
						DEVICE_ID, frame->sender, frame->frame_id, data_buffer,
								0)) {
							UART_TX_FSend("%s", response_buffer);
						}
					}
				}
			}
		}

		if (error) {
			if (build_response_frame(response_buffer, response_size, DEVICE_ID,
					frame->sender, frame->frame_id, NULL, error)) {
				UART_TX_FSend("%s", response_buffer);
			}
		}
	}
		break;


	case SETINT_CMD:
	{
		if (frame->params_len != PARAM_LEN_SETINT) {
			error = WRLEN;
		} else {
			int new_interval = convert_char_to_int(frame->params);
			if (new_interval <= 0) {
				error = WRCMD;
			} else {
				uint16_t integration_time = get_integration_time_ms(current_time_index);
				if ((uint32_t)new_interval <= integration_time) {
					error = WRTIME;
				} else {
					timer_interval = new_interval;
					if (build_response_frame(response_buffer, response_size,
					DEVICE_ID, frame->sender, frame->frame_id, RESP_OK, 0)) {
						UART_TX_FSend("%s", response_buffer);
					}
				}
			}
		}

		if (error) {
			if (build_response_frame(response_buffer, response_size, DEVICE_ID,
					frame->sender, frame->frame_id, NULL, error)) {
				UART_TX_FSend("%s", response_buffer);
			}
		}
	}
		break;

	case GETINT_CMD:
	{
		sprintf(data_buffer, INT_PREFIX "%05lu", timer_interval);
		if (build_response_frame(response_buffer, response_size, DEVICE_ID,
				frame->sender, frame->frame_id, data_buffer, 0)) {
			UART_TX_FSend("%s", response_buffer);
		}
	}
		break;

	case SETGAIN_CMD:
	{
		if (frame->params_len != PARAM_LEN_SETGAIN) {
			error = WRLEN;
		} else {
			char gain_char = frame->params[0];
			if (gain_char >= '0' && gain_char <= '3') {
				current_gain_index = gain_char - '0';
				TCS34725_WriteReg(&hi2c1, TCS34725_CONTROL, GAIN_TABLE[current_gain_index]);
				if (build_response_frame(response_buffer, response_size,
				DEVICE_ID, frame->sender, frame->frame_id, RESP_OK, 0)) {
					UART_TX_FSend("%s", response_buffer);
				}
			} else {
				error = WRCMD;
			}
		}

		if (error) {
			if (build_response_frame(response_buffer, response_size, DEVICE_ID,
					frame->sender, frame->frame_id, NULL, error)) {
				UART_TX_FSend("%s", response_buffer);
			}
		}
	}
		break;

	case GETGAIN_CMD:
	{
		sprintf(data_buffer, GAIN_PREFIX "%01u", current_gain_index);
		if (build_response_frame(response_buffer, response_size, DEVICE_ID,
				frame->sender, frame->frame_id, data_buffer, 0)) {
			UART_TX_FSend("%s", response_buffer);
		}
	}
		break;

	case SETTIME_CMD:
	{
		if (frame->params_len != PARAM_LEN_SETTIME) {
			error = WRLEN;
		} else {
			char time_char = frame->params[0];
			if (time_char >= '0' && time_char <= '4') {
				uint8_t new_time_index = time_char - '0';
				uint16_t new_integration_time = get_integration_time_ms(new_time_index);
				if (timer_interval <= new_integration_time) {
					error = WRTIME;
				} else {
					current_time_index = new_time_index;
					TCS34725_WriteReg(&hi2c1, TCS34725_ATIME, TIME_TABLE[new_time_index]);
					if (build_response_frame(response_buffer, response_size,
					DEVICE_ID, frame->sender, frame->frame_id, RESP_OK, 0)) {
						UART_TX_FSend("%s", response_buffer);
					}
				}
			} else {
				error = WRCMD;
			}
		}

		if (error) {
			if (build_response_frame(response_buffer, response_size, DEVICE_ID,
					frame->sender, frame->frame_id, NULL, error)) {
				UART_TX_FSend("%s", response_buffer);
			}
		}
	}
		break;

	case GETTIME_CMD:
	{
		sprintf(data_buffer, TIME_PREFIX "%01u", current_time_index);
		if (build_response_frame(response_buffer, response_size, DEVICE_ID,
				frame->sender, frame->frame_id, data_buffer, 0)) {
			UART_TX_FSend("%s", response_buffer);
		}
	}
		break;

	case SETLED_CMD:
	{
		if (frame->params_len != PARAM_LEN_SETLED) {
			error = WRLEN;
		} else {
			char led_char = frame->params[0];
			if (led_char == '0' || led_char == '1') {
				led_state = led_char - '0';
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, led_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
				if (build_response_frame(response_buffer, response_size,
				DEVICE_ID, frame->sender, frame->frame_id, RESP_OK, 0)) {
					UART_TX_FSend("%s", response_buffer);
				}
			} else {
				error = WRCMD;
			}
		}

		if (error) {
			if (build_response_frame(response_buffer, response_size, DEVICE_ID,
					frame->sender, frame->frame_id, NULL, error)) {
				UART_TX_FSend("%s", response_buffer);
			}
		}
	}
		break;

	case GETLED_CMD:
	{
		GPIO_PinState actual_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3);
		uint8_t actual_led_state = (actual_state == GPIO_PIN_SET) ? 1 : 0;

		sprintf(data_buffer, LED_PREFIX "%01u", actual_led_state);
		if (build_response_frame(response_buffer, response_size, DEVICE_ID,
				frame->sender, frame->frame_id, data_buffer, 0)) {
			UART_TX_FSend("%s", response_buffer);
		}
	}
	break;

	default:
		if (build_response_frame(response_buffer, response_size, DEVICE_ID,
				frame->sender, frame->frame_id, NULL, WRCMD)) {
			UART_TX_FSend("%s", response_buffer);
		}
		break;
	}
}
