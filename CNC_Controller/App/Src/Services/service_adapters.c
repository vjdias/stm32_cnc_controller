#include "Services/service_adapters.h"
#include "Services/Motion/motion_service.h"
#include "Services/Home/home_service.h"
#include "Services/Probe/probe_service.h"
#include "Services/Led/led_service.h"
#include "Services/Test/test_spi_service.h"

// Funções estáticas de adaptação compatíveis com os callbacks do roteador
static void h_move_queue_add(router_t *r, const uint8_t *f, uint32_t l) {
	(void) r;
	motion_on_move_queue_add(f, l);
}
static void h_move_queue_status(router_t *r, const uint8_t *f, uint32_t l) {
	(void) r;
	motion_on_move_queue_status(f, l);
}
static void h_start_move(router_t *r, const uint8_t *f, uint32_t l) {
	(void) r;
	motion_on_start_move(f, l);
}
static void h_move_home(router_t *r, const uint8_t *f, uint32_t l) {
	(void) r;
	home_on_move_home(f, l);
}
static void h_move_probe_level(router_t *r, const uint8_t *f, uint32_t l) {
	(void) r;
	probe_on_move_probe_level(f, l);
}
static void h_move_end(router_t *r, const uint8_t *f, uint32_t l) {
	(void) r;
	motion_on_move_end(f, l);
}
static void h_led_ctrl(router_t *r, const uint8_t *f, uint32_t l) {
	(void) r;
	led_on_led_ctrl(f, l);
}
static void h_fpga_status(router_t *r, const uint8_t *f, uint32_t l) {
	(void) r;
	(void) f;
	(void) l; /* opcional */
}

static void h_test_hello(router_t *r, const uint8_t *f, uint32_t l) {
	(void) r;
	//test_spi_on_hello(f, l);
}

void services_register_handlers(router_handlers_t *h) {
	if (!h)
		return;
	h->on_move_queue_add = h_move_queue_add;
	h->on_move_queue_status = h_move_queue_status;
	h->on_start_move = h_start_move;
	h->on_move_home = h_move_home;
	h->on_move_probe_level = h_move_probe_level;
	h->on_move_end = h_move_end;
	h->on_led_ctrl = h_led_ctrl;
	h->on_fpga_status = h_fpga_status;
	h->on_test_hello = h_test_hello;
}
