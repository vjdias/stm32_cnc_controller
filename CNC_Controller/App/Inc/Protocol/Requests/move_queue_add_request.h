// MOVE_QUEUE_ADD (42 bytes) — 0x01
//
// Notas sobre PID (formato atual e futura extensão):
// - O firmware consome kp/ki/kd como inteiros de 16 bits já escalados
//   (formato fixo Q8 — ver MOTION_PI_SHIFT=8 no motion_service.c). Não há
//   multiplicação interna por 256; o emissor deve enviar o valor inteiro
//   esperado pelo firmware.
// - FUTURO: caso seja necessário aceitar "floats puros" (ex.: 3.1300) no
//   protocolo, considerar um novo request alternativo com campos de 32 bits
//   para os ganhos (ou uma escala fixa de 10^4). A referência para a escolha
//   de escala é o serviço de LED, que usa centi-Hz para representar frequência
//   com 2 casas decimais. Para PID, uma escala de 10^4 permitiria 4 casas.
//   Essa mudança exigiria um novo REQ_* para manter compatibilidade.
#pragma once
#include <stdint.h>
#include "../frame_defs.h"

typedef struct {
	uint8_t frameId;
	uint8_t dirMask;
	uint16_t vx;
	uint32_t sx;
	uint16_t vy;
	uint32_t sy;
	uint16_t vz;
	uint32_t sz;
	uint16_t kp_x, ki_x, kd_x;
	uint16_t kp_y, ki_y, kd_y;
	uint16_t kp_z, ki_z, kd_z;
} move_queue_add_req_t;

int move_queue_add_req_decoder(const uint8_t *raw, uint32_t len,
		move_queue_add_req_t *out);
int move_queue_add_req_encoder(const move_queue_add_req_t *in, uint8_t *raw,
		uint32_t len);
uint8_t move_queue_add_req_calc_parity(const move_queue_add_req_t *in); // retorna o bit no bit0
int move_queue_add_req_check_parity(const uint8_t *raw, uint32_t len);
int move_queue_add_req_set_parity(uint8_t *raw, uint32_t len);
move_queue_add_req_t move_queue_add_req_make_default(void);

