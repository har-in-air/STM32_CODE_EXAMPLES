#ifndef HID_REPORT_H_
#define HID_REPORT_H_

#include <stdint.h>

typedef struct {
	int8_t      dx; // dx
	int8_t      dy; // dy
	uint8_t     buttons; // R = buttons[1], L = buttons[0]
} __attribute__((__packed__)) HID_REPORT_t;

void hid_write_mouse_report();

#endif /* HID_REPORT_H_ */

