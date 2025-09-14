#include "services/service_adapters.h"
#include "services/motion/motion_service.h"
#include "services/home/home_service.h"
#include "services/probe/probe_service.h"
#include "services/led/led_service.h"

// Static adapter functions matching router callbacks
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
}
