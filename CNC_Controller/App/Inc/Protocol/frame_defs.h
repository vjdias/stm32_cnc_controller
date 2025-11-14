// Definições de frame alinhadas com o README (IDs, headers, tails)
#pragma once
#include <stdint.h>

// Bytes de enquadramento
enum {
	REQ_HEADER = 0xAA, REQ_TAIL = 0x55, RESP_HEADER = 0xAB, RESP_TAIL = 0x54,
};

// Tipos de mensagens de requisição (STM32 recebe)
typedef enum {
	REQ_MOVE_QUEUE_ADD = 0x01,
	REQ_MOVE_QUEUE_STATUS = 0x02,
	REQ_START_MOVE = 0x03,
	REQ_MOVE_HOME = 0x04,
        REQ_MOVE_PROBE_LEVEL = 0x05,
        REQ_MOVE_END = 0x06,
        REQ_LED_CTRL = 0x07,
        REQ_STM32_STATUS = 0x20,
        REQ_SET_ORIGIN = 0x24,
        REQ_ENCODER_STATUS = 0x25,
        REQ_SET_MICROSTEPS = 0x26,
        REQ_SET_MICROSTEPS_AX = 0x27,
        REQ_TEST_HELLO = 0x68,
        REQ_MOTION_AUTO_FRICTION = 0x69,
} req_msg_type_t;

// Tipos de mensagens de resposta (STM32 envia)
typedef enum {
        RESP_MOVE_QUEUE_ADD_ACK = 0x01,
	RESP_MOVE_QUEUE_STATUS = 0x02,
	RESP_START_MOVE = 0x03,
	RESP_MOVE_HOME = 0x04,
        RESP_MOVE_PROBE_LEVEL = 0x05,
        RESP_MOVE_END = 0x06,
        RESP_LED_CTRL = 0x07,
        RESP_FPGA_STATUS = 0x20,
        RESP_HOME_STATUS = 0x21,
        RESP_SET_ORIGIN = 0x24,
        RESP_ENCODER_STATUS = 0x25,
        RESP_SET_MICROSTEPS = 0x26,
        RESP_TEST_HELLO = 0x68,
        RESP_MOTION_AUTO_FRICTION = 0x69,
} resp_msg_type_t;

// =====================
// Conjunto genérico de auxiliares
// =====================

// XOR sobre bytes (paridade byte a byte)
static inline uint8_t xor_reduce_bytes(const uint8_t *p, uint32_t n) {
	uint8_t x = 0;
	for (uint32_t i = 0; i < n; ++i)
		x ^= p[i];
	return x;
}

// Redução de XOR bit a bit (retorna um único bit de paridade no bit0)
static inline uint8_t xor_bit_reduce_bytes(const uint8_t *p, uint32_t n) {
	uint8_t x = 0;
	for (uint32_t i = 0; i < n; ++i)
		x ^= p[i];
	x ^= (uint8_t) (x >> 4);
	x ^= (uint8_t) (x >> 2);
	x ^= (uint8_t) (x >> 1);
	return (uint8_t) (x & 0x1);
}

// Verifica/ajusta a paridade por byte em um intervalo: valor em raw[parity_index]
static inline int check_parity_byte(const uint8_t *raw, uint32_t start,
		uint32_t count, uint32_t parity_index) {
	if (!raw)
		return 0;
	return xor_reduce_bytes(raw + start, count) == raw[parity_index];
}
static inline int set_parity_byte(uint8_t *raw, uint32_t start, uint32_t count,
		uint32_t parity_index) {
	if (!raw)
		return -1;
	raw[parity_index] = xor_reduce_bytes(raw + start, count);
	return 0;
}

// Verifica/ajusta a paridade reduzida a um bit: LSB de raw[parity_index]
static inline int check_parity_bit(const uint8_t *raw, uint32_t start,
		uint32_t count, uint32_t parity_index) {
	if (!raw)
		return 0;
	return ((raw[parity_index] & 0x1)
			== xor_bit_reduce_bytes(raw + start, count));
}
static inline int set_parity_bit(uint8_t *raw, uint32_t start, uint32_t count,
		uint32_t parity_index) {
	if (!raw)
		return -1;
	raw[parity_index] = (uint8_t) (xor_bit_reduce_bytes(raw + start, count)
			& 0x1);
	return 0;
}

// Validação genérica de header/tail
static inline int has_header_tail(const uint8_t *raw, uint32_t len,
		uint8_t header, uint8_t tail) {
	return raw && len >= 2 && raw[0] == header && raw[len - 1] == tail;
}

// Leituras/escritas big-endian (formato no fio)
static inline uint16_t be16_read(const uint8_t *p) {
	return (uint16_t) ((((uint16_t) p[0]) << 8) | p[1]);
}
static inline uint32_t be32_read(const uint8_t *p) {
	return (((uint32_t) p[0] << 24) | ((uint32_t) p[1] << 16)
			| ((uint32_t) p[2] << 8) | (uint32_t) p[3]);
}
static inline void be16_write(uint8_t *p, uint16_t v) {
	p[0] = (uint8_t) (v >> 8);
	p[1] = (uint8_t) v;
}
static inline void be32_write(uint8_t *p, uint32_t v) {
	p[0] = (uint8_t) (v >> 24);
	p[1] = (uint8_t) (v >> 16);
	p[2] = (uint8_t) (v >> 8);
	p[3] = (uint8_t) v;
}

// =====================
// Códigos de retorno padronizados
// =====================
// Usados por encoders/decoders/validadores para manter uniformidade
typedef enum {
        PROTO_OK = 0,   // Sucesso
        PROTO_WARN = 1,   // Condição não fatal (uso opcional)

        PROTO_ERR_ARG = -1,  // Argumento(s) ou tamanho inválido(s)
        PROTO_ERR_FRAME = -2,  // Cabeçalho/tipo incompatível
        PROTO_ERR_ALLOC = -3,  // Falha de alocação
        PROTO_ERR_RANGE = -4,  // Faixa/overflow/buffer insuficiente
        PROTO_ERR_PARITY = -5,  // Erro de paridade/checksum
} proto_result_t;

// Auxiliares para testar categorias de resultado
#define PROTO_SUCCEEDED(x)   ((x) >= 0)
#define PROTO_FAILED(x)      ((x) < 0)
#define PROTO_IS_WARN(x)     ((x) > 0)

// =====================
// Auxiliares de frame (init/tail)
// =====================
static inline void req_init(uint8_t *raw, req_msg_type_t type) {
	raw[0] = REQ_HEADER;
	raw[1] = (uint8_t) type;
}
static inline void resp_init(uint8_t *raw, resp_msg_type_t type) {
	raw[0] = RESP_HEADER;
	raw[1] = (uint8_t) type;
}
static inline void req_set_tail(uint8_t *raw, uint32_t tail_index) {
	raw[tail_index] = REQ_TAIL;
}
static inline void resp_set_tail(uint8_t *raw, uint32_t tail_index) {
	raw[tail_index] = RESP_TAIL;
}

// =====================
// Validadores de frame
// =====================
// Garante comprimento mínimo, header/tail corretos e tipo esperado
static inline int frame_expect_req(const uint8_t *raw, uint32_t len,
		req_msg_type_t type, uint32_t min_len) {
	if (!raw || len < min_len)
		return PROTO_ERR_ARG;
	if (!has_header_tail(raw, len, REQ_HEADER, REQ_TAIL)
			|| raw[1] != (uint8_t) type)
		return PROTO_ERR_FRAME;
	return PROTO_OK;
}
static inline int frame_expect_resp(const uint8_t *raw, uint32_t len,
		resp_msg_type_t type, uint32_t min_len) {
	if (!raw || len < min_len)
		return PROTO_ERR_ARG;
	if (!has_header_tail(raw, len, RESP_HEADER, RESP_TAIL)
			|| raw[1] != (uint8_t) type)
		return PROTO_ERR_FRAME;
	return PROTO_OK;
}

// =====================
// Wrappers de paridade (intervalo 1..N)
// =====================
// Assume que a paridade cobre os bytes do índice 1 (tipo) até last_index inclusive
static inline int parity_set_byte_1N(uint8_t *raw, uint32_t last_index,
		uint32_t parity_index) {
	return set_parity_byte(raw, 1, last_index, parity_index);
}
static inline int parity_check_byte_1N(const uint8_t *raw, uint32_t last_index,
		uint32_t parity_index) {
	return check_parity_byte(raw, 1, last_index, parity_index);
}
static inline int parity_set_bit_1N(uint8_t *raw, uint32_t last_index,
		uint32_t parity_index) {
	return set_parity_bit(raw, 1, last_index, parity_index);
}
static inline int parity_check_bit_1N(const uint8_t *raw, uint32_t last_index,
		uint32_t parity_index) {
	return check_parity_bit(raw, 1, last_index, parity_index);
}
