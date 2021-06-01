#ifndef USBD_DRIVER_H_
#define USBD_DRIVER_H_

#include "cmsis/device/stm32f4xx.h"
#include "usb/usb_standards.h"

#define USB_OTG_HS_GLOBAL ((USB_OTG_GlobalTypeDef *)(USB_OTG_HS_PERIPH_BASE + USB_OTG_GLOBAL_BASE))
#define USB_OTG_HS_DEVICE ((USB_OTG_DeviceTypeDef *)(USB_OTG_HS_PERIPH_BASE + USB_OTG_DEVICE_BASE))
#define USB_OTG_HS_PCGCCTL ((__IO uint32_t *)(USB_OTG_HS_PERIPH_BASE + USB_OTG_PCGCCTL_BASE)) // Power and clock gating control register

/** \brief Returns the structure contains the registers of a specific IN endpoint.
 * \param endpoint_number The number of the IN endpoint we want to access its registers.
 */
inline static USB_OTG_INEndpointTypeDef * IN_ENDPOINT(uint8_t endpoint_number)
{
    return (USB_OTG_INEndpointTypeDef *)(USB_OTG_HS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + (endpoint_number * 0x20));
}

/** \brief Returns the structure contains the registers of a specific OUT endpoint.
 * \param endpoint_number The number of the OUT endpoint we want to access its registers.
 */
inline static USB_OTG_OUTEndpointTypeDef * OUT_ENDPOINT(uint8_t endpoint_number)
{
    return (USB_OTG_OUTEndpointTypeDef *)(USB_OTG_HS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + (endpoint_number * 0x20));
}

inline static __IO uint32_t *FIFO(uint8_t endpoint_number)
{
    return (__IO uint32_t *)(USB_OTG_HS_PERIPH_BASE + USB_OTG_FIFO_BASE + (endpoint_number * 0x1000));
}

// Total count of IN or OUT endpoints. Note for OTG_HS  its 6, for OTG_FS its 4
#define ENDPOINT_COUNT 6

// USB driver functions exposed to USB framework.
typedef struct {
	void (*initialize_core)();
	void (*initialize_gpio_pins)();
	void (*set_device_address)(uint8_t address);
	void (*connect)();
	void (*disconnect)();
	void (*flush_rxfifo)();
	void (*flush_txfifo)(uint8_t endpoint_number);
	void (*configure_in_endpoint)(uint8_t endpoint_number, USB_ENDPOINT_TYPE_e endpoint_type, uint16_t endpoint_size);
	void (*read_packet)(void  *buffer, uint16_t size);
	void (*write_packet)(uint8_t endpoint_number, const void *buffer, uint16_t size);
	void (*poll)();
	// ToDO Add pointers to the other driver functions.
} USB_DRIVER_t;

extern const USB_DRIVER_t USB_Driver;
extern USB_EVENTS_t USB_Events;

#endif /* USBD_DRIVER_H_ */
