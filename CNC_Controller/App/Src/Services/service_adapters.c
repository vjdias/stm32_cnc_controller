#include "Services/service_adapters.h"
#include "Services/Led/led_service.h"

static void h_led_ctrl(router_t *r, const uint8_t *f, uint32_t l) {
        (void) r;
        led_on_led_ctrl(f, l);
}

void services_register_handlers(router_handlers_t *h) {
        if (!h)
                return;
        // Mantemos apenas o manipulador essencial para o serviço de LED.
        h->on_led_ctrl = h_led_ctrl;
}
