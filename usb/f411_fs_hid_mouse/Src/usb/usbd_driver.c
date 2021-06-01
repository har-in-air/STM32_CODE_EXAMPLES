#include <string.h>
#include "usb/usbd_driver.h"
#include "util/logger.h"

static void usbd_configure_in_endpoint(uint8_t endpoint_number, USB_ENDPOINT_TYPE_e endpoint_type, uint16_t endpoint_size);
static void usbd_configure_endpoint0(uint8_t endpoint_size);
static void usbd_deconfigure_endpoint(uint8_t endpoint_number);
static void usbd_usbrst_handler();
static void usbd_global_int_status_handler();
static void usbd_disconnect();
static void usbd_connect();
static void usbd_initialize_gpio_pins();
static void usbd_initialize_core();
static void usbd_refresh_fifo_start_addresses();
static void usbd_configure_rxfifo_size(uint16_t sizeBytes);
static void usbd_configure_txfifo_size(uint8_t endpoint_number, uint16_t sizeBytes);
static void usbd_flush_rxfifo();
static void usbd_flush_txfifo(uint8_t endpoint_number);
static void usbd_write_packet(uint8_t endpoint_number, const void *buffer, uint16_t sizeBytes);
static void usbd_read_packet(void *buffer, uint16_t sizeBytes);
static void usbd_enumdne_handler();
static void usbd_rxflvl_handler();
static void usbd_oepint_handler();
static void usbd_iepint_handler();
static void usbd_set_device_address(uint8_t address);

// external driver interface
const USB_DRIVER_t USB_Driver = {
	.initialize_core = &usbd_initialize_core,
	.initialize_gpio_pins = &usbd_initialize_gpio_pins,
	.set_device_address = &usbd_set_device_address,
	.connect = &usbd_connect,
	.disconnect = &usbd_disconnect,
	.flush_rxfifo = &usbd_flush_rxfifo,
	.flush_txfifo = &usbd_flush_txfifo,
	.configure_in_endpoint = &usbd_configure_in_endpoint,
	.read_packet = &usbd_read_packet,
	.write_packet = &usbd_write_packet,
	.poll = &usbd_global_int_status_handler // polls status bits set by interrupts
	};


static void usbd_initialize_gpio_pins() {
	// Enables the clock for GPIOA
	SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);
	// Configures PA11 and PA12 as USB FS pins DP- and DP+ (AF10)
	// use AFRH (AFR[1]) as pins 0-7 are assigned to AFRL, 8-15 in AFRH
	MODIFY_REG(GPIOA->AFR[1],
		GPIO_AFRH_AFSEL11 | GPIO_AFRH_AFSEL12,
		_VAL2FLD(GPIO_AFRH_AFSEL11, 10) | _VAL2FLD(GPIO_AFRH_AFSEL12, 10)
		);
	// Configures USB pins to work in alternate function mode, mode 2 (0= input, 1= output).
	MODIFY_REG(GPIOA->MODER,
		GPIO_MODER_MODER11 | GPIO_MODER_MODER12,
		_VAL2FLD(GPIO_MODER_MODER11, 2) | _VAL2FLD(GPIO_MODER_MODER12, 2)
		);
	// PA9 is VBUS sense (input), use a 5.6k+10k resistor divider from VBUS-GND to make the sensed voltage 3.3V level
	MODIFY_REG(GPIOA->MODER,
			GPIO_MODER_MODER9,
			_VAL2FLD(GPIO_MODER_MODER9, 0)
		);
	}


static void usbd_initialize_core() {
	// Enable the clock for USB core.
	SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_OTGFSEN);

	// USB_OTG_GUSBCFG_FDMOD = 1 : Force the USB core to run in device mode irrespective of otg_fs_id pin
	// USB_OTG_GUSBCFG_PHYSEL = 1 : Use full-speed PHY
	// USB_OTG_GUSBCFG_TRDT = 6 : RM0090 pg1276, for AHB clock > 32MHz, min TRDT = 6
	MODIFY_REG(USB_OTG_FS->GUSBCFG,
		USB_OTG_GUSBCFG_FDMOD | USB_OTG_GUSBCFG_PHYSEL | USB_OTG_GUSBCFG_TRDT,
		USB_OTG_GUSBCFG_FDMOD | USB_OTG_GUSBCFG_PHYSEL | _VAL2FLD(USB_OTG_GUSBCFG_TRDT, 0x06)
	);

	// USB_OTG_DCFG_DSPD = 3 : Configure the device to run in full speed mode.
	MODIFY_REG(USB_OTG_FS_DEVICE->DCFG,
		USB_OTG_DCFG_DSPD,
		_VAL2FLD(USB_OTG_DCFG_DSPD, 0x03)
	);

	// disable no vbus sensing
	CLEAR_BIT(USB_OTG_FS->GCCFG, USB_OTG_GCCFG_NOVBUSSENS);
	// disable A device vbus sensing
	CLEAR_BIT(USB_OTG_FS->GCCFG, USB_OTG_GCCFG_VBUSASEN);
	// Enable B-device vbus sensing
	SET_BIT(USB_OTG_FS->GCCFG, USB_OTG_GCCFG_VBUSBSEN);

	// disable VBUS sensing, assume device is connected at power-on to VBUS
	//SET_BIT(USB_OTG_FS_DEVICE->DCTL, USB_OTG_DCTL_SDIS);
    //SET_BIT(USB_OTG_FS->GCCFG, USB_OTG_GCCFG_NOVBUSSENS);
    //CLEAR_BIT(USB_OTG_FS->GCCFG, USB_OTG_GCCFG_VBUSBSEN);
    //CLEAR_BIT(USB_OTG_FS->GCCFG, USB_OTG_GCCFG_VBUSASEN);

	// Unmasks the main USB core interrupts.
    // See usbd_global_int_status_handler
	SET_BIT(USB_OTG_FS->GINTMSK,
		USB_OTG_GINTMSK_USBRST | USB_OTG_GINTMSK_ENUMDNEM | USB_OTG_GINTMSK_SOFM |
		USB_OTG_GINTMSK_USBSUSPM | USB_OTG_GINTMSK_WUIM | USB_OTG_GINTMSK_IEPINT |
		USB_OTG_GINTSTS_OEPINT | USB_OTG_GINTMSK_RXFLVLM
	);

	// Clears all pending core interrupts.
	// rm0090 pg 1279
	WRITE_REG(USB_OTG_FS->GINTSTS, 0xFFFFFFFF);


	// Unmasks USB global interrupt.
	SET_BIT(USB_OTG_FS->GAHBCFG, USB_OTG_GAHBCFG_GINT);

	// Unmasks transfer completed interrupts for all endpoints.
	SET_BIT(USB_OTG_FS_DEVICE->DOEPMSK, USB_OTG_DOEPMSK_XFRCM);
	SET_BIT(USB_OTG_FS_DEVICE->DIEPMSK, USB_OTG_DIEPMSK_XFRCM);
	}



// Turn on usb transceiver only when USB is connected, to save power
static void usbd_connect() {
	// Powers the transceivers on.
    SET_BIT(USB_OTG_FS->GCCFG, USB_OTG_GCCFG_PWRDWN);
	// Connects the device to the bus.
    CLEAR_BIT(USB_OTG_FS_DEVICE->DCTL, USB_OTG_DCTL_SDIS);
	}

// Disconnects the USB device from the bus, turns off transceiver to save power
static void usbd_disconnect() {
	// Disconnects the device from the bus.
	SET_BIT(USB_OTG_FS_DEVICE->DCTL, USB_OTG_DCTL_SDIS);
	// Powers the transceivers off.
	CLEAR_BIT(USB_OTG_FS->GCCFG, USB_OTG_GCCFG_PWRDWN);
	}


// Handles the USB core interrupts.
static void usbd_global_int_status_handler() {
	volatile uint32_t global_int_status = USB_OTG_FS_GLOBAL->GINTSTS;

	// usb reset signal received from host
	// device needs to reset usb communications state into default
	// configuration state that the host is aware of
	if (global_int_status & USB_OTG_GINTSTS_USBRST)	{
		log_debug("usbrst handler");
		usbd_usbrst_handler();
		// Clears the interrupt.
		SET_BIT(USB_OTG_FS_GLOBAL->GINTSTS, USB_OTG_GINTSTS_USBRST);
		}
	// enumeration done
	else if (global_int_status & USB_OTG_GINTSTS_ENUMDNE)	{
		log_debug("enumdone handler");
		usbd_enumdne_handler();
		// Clears the interrupt.
		SET_BIT(USB_OTG_FS_GLOBAL->GINTSTS, USB_OTG_GINTSTS_ENUMDNE);
		}
	// rx fifo not empty handler
	//  rx fifo may have some meta-data (e.g. how many data bytes received from host)
	// as well as actual data from host.
	else if (global_int_status & USB_OTG_GINTSTS_RXFLVL)	{
		log_debug("rxfifo not empty handler");
		usbd_rxflvl_handler();
		// Clears the interrupt.
		SET_BIT(USB_OTG_FS_GLOBAL->GINTSTS, USB_OTG_GINTSTS_RXFLVL);
		}
	// IN endpoint interrupt (check to find out which IN endpoint and which interrupt)
	else if (global_int_status & USB_OTG_GINTSTS_IEPINT)	{
		log_debug("in endpoint handler");
		usbd_iepint_handler();
		// Clears the interrupt.
		SET_BIT(USB_OTG_FS_GLOBAL->GINTSTS, USB_OTG_GINTSTS_IEPINT);
		}
	// OUT endpoint interrupt
	else if (global_int_status & USB_OTG_GINTSTS_OEPINT)	{
		log_debug("out endpoint handler");
		usbd_oepint_handler();
		// Clears the interrupt.
		SET_BIT(USB_OTG_FS_GLOBAL->GINTSTS, USB_OTG_GINTSTS_OEPINT);
		}
	USB_Events.on_usb_polled();
	}


static void usbd_usbrst_handler() {
	log_info("USB reset signal detected.");
	// note that interrupt unmasking iss done in core_initialize
	for (uint8_t inx = 0; inx <= USBD_ENDPOINT_COUNT; inx++)	{
		usbd_deconfigure_endpoint(inx);
		}
    // non-driver related reset handling code in framework
	USB_Events.on_usb_reset_received();
	}


// Deconfigures IN and OUT endpoints of a specific endpoint number.
// Note : EP 0 cannot be disabled, OUT EP0 SETUP packets cannot be NAK'ed
// as host must always be able to send control packets to device
static void usbd_deconfigure_endpoint(uint8_t endpoint_number) {
    USB_OTG_INEndpointTypeDef *in_endpoint = IN_ENDPOINT(endpoint_number);
    USB_OTG_OUTEndpointTypeDef *out_endpoint = OUT_ENDPOINT(endpoint_number);

	// Masks all interrupts of the targeted IN and OUT endpoints.
	CLEAR_BIT(USB_OTG_FS_DEVICE->DAINTMSK,
		(1 << endpoint_number) | (1 << 16 << endpoint_number)
		);

	// clear-by-set
	// Clears all interrupts & status bits of the input endpoint (respect the reserved fields).
	// ref : RM0090, pg 1318
	// bit 11 is a status field
	//SET_BIT(in_endpoint->DIEPINT, 0x287B);
	// clear all interrupts and status bits for output endpoint
    //SET_BIT(out_endpoint->DOEPINT, 0x31BB);

	SET_BIT(in_endpoint->DIEPINT, 0x29FF);
    SET_BIT(out_endpoint->DOEPINT, 0x715F);

	// Disables the endpoints if possible.
    if (in_endpoint->DIEPCTL & USB_OTG_DIEPCTL_EPENA)    {
		// Disables endpoint transmission.
		SET_BIT(in_endpoint->DIEPCTL, USB_OTG_DIEPCTL_EPDIS);
    	}

	// Deactivates the endpoint.
	CLEAR_BIT(in_endpoint->DIEPCTL, USB_OTG_DIEPCTL_USBAEP);

	// out endpoint 0 cannot be disabled or deactivated
    if (endpoint_number != 0)    {
		if (out_endpoint->DOEPCTL & USB_OTG_DOEPCTL_EPENA)		{
			// Disables endpoint transmission.
			SET_BIT(out_endpoint->DOEPCTL, USB_OTG_DOEPCTL_EPDIS);
			}

		// Deactivates the endpoint.
		CLEAR_BIT(out_endpoint->DOEPCTL, USB_OTG_DOEPCTL_USBAEP);
    	}

	// Flushes the FIFOs.
	usbd_flush_txfifo(endpoint_number);
	// not necessary ?
	usbd_flush_rxfifo();
	}


// endpoint_size is max packet size for endpoint0 (for FS the options are 8, 16, 32, 64)
// same size is used for IN and OUT
static void usbd_configure_endpoint0(uint8_t endpoint_size) {
	// every IN and OUT endpoint has its own mask interrupt in DAINTMSK
	// 0-15 for IN EPs, 16-32 for OUT EPs
	// Unmask IN EP (bit0) and OUT EP (bit16) interrupts for endpoint0
	SET_BIT(USB_OTG_FS_DEVICE->DAINTMSK, (1 << 0) | (1 << 16));

	// Configures the maximum packet size
	// activates the endpoint USBAEP (specifies this EP is to be used)
	// Set NAK (endpoint cannot send data)
	// EP is not enabled yet
	MODIFY_REG(IN_ENDPOINT(0)->DIEPCTL,
		USB_OTG_DIEPCTL_MPSIZ,
		USB_OTG_DIEPCTL_USBAEP | _VAL2FLD(USB_OTG_DIEPCTL_MPSIZ, endpoint_size) | USB_OTG_DIEPCTL_SNAK
		);

	// max packet size for OUT is same as for IN
	// OUT 0 is always activated (host must always be able to transmit to device on endpoint 0)
	// Clears NAK, and enables endpoint data reception.
	SET_BIT(OUT_ENDPOINT(0)->DOEPCTL,
		USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK
		);

	// Note: 64 bytes is the maximum packet size for full speed USB devices.
	usbd_configure_rxfifo_size(64);
	usbd_configure_txfifo_size(0, endpoint_size);
	}

// note endpoint 0 can be reconfigured after enumeration to be another endpoint type
// but we are keeping things simple by leaving it as a control endpoint
// note : it is possible to assign a txfifo number that is different than the endpoint number !!
// recommended to use the same indices
static void usbd_configure_in_endpoint(uint8_t endpoint_number, USB_ENDPOINT_TYPE_e endpoint_type, uint16_t endpoint_size) {
	// Unmasks IN endpoint interrupt
	SET_BIT(USB_OTG_FS_DEVICE->DAINTMSK, 1 << endpoint_number);

	// IEPCTL : IN endpoint control register
	// Activate IN endpoint, set endpoint handshake to NAK (not ready to send data), set DATA0 packet identifier,
	// configure endpoint type, maximum packet size, assign it a TxFIFO number ==  endpoint_number to avoid confusion
	MODIFY_REG(IN_ENDPOINT(endpoint_number)->DIEPCTL,
		USB_OTG_DIEPCTL_MPSIZ | USB_OTG_DIEPCTL_EPTYP | USB_OTG_DIEPCTL_TXFNUM,
		USB_OTG_DIEPCTL_USBAEP | _VAL2FLD(USB_OTG_DIEPCTL_MPSIZ, endpoint_size) | USB_OTG_DIEPCTL_SNAK |
		_VAL2FLD(USB_OTG_DIEPCTL_EPTYP, endpoint_type) | _VAL2FLD(USB_OTG_DIEPCTL_TXFNUM, endpoint_number) | USB_OTG_DIEPCTL_SD0PID_SEVNFRM
		);

	usbd_configure_txfifo_size(endpoint_number, endpoint_size);
	}


// note : currently no function for configuring out endpoints as not required for this course !!

// Updates the start addresses of all FIFOs according to the size of each FIFO.
// note : fifo depths are specified in words, but the start addresses are specified as byte addresses
// (so the * 4 needed)
// must be called after any fifo (rx or tx) size is configured
static void usbd_refresh_fifo_start_addresses() {
	// The first txfifo start address begins after the region of RxFIFO which starts at 0
	uint16_t start_address = _FLD2VAL(USB_OTG_GRXFSIZ_RXFD, USB_OTG_FS->GRXFSIZ) * 4;

	// Updates the start address of the TxFIFO0.
	MODIFY_REG(USB_OTG_FS->DIEPTXF0_HNPTXFSIZ,
		USB_OTG_TX0FSA,
		_VAL2FLD(USB_OTG_TX0FSA, start_address)
		);

	// The next start address is after where the last TxFIFO ends.
	start_address += _FLD2VAL(USB_OTG_TX0FD, USB_OTG_FS->DIEPTXF0_HNPTXFSIZ) * 4;

	// Updates the start addresses of the rest TxFIFOs as TXFIFO0 register is not contiguous,
	for (uint8_t txfifo_number = 0; txfifo_number < USBD_ENDPOINT_COUNT - 1; txfifo_number++) {
		MODIFY_REG(USB_OTG_FS->DIEPTXF[txfifo_number],
				USB_OTG_DIEPTXF_INEPTXSA,
			_VAL2FLD(USB_OTG_DIEPTXF_INEPTXSA, start_address)
			);
		start_address += _FLD2VAL(USB_OTG_NPTXFD, USB_OTG_FS->DIEPTXF[txfifo_number]) * 4;
		}
	}


// Configures the RxFIFO for all OUT endpoints.
// size The size of the largest OUT endpoint in bytes.
// The RxFIFO is shared between all OUT endpoints.
// meta-data status info is pushed to rxfifo before actual data
// e.g. how many bytes will follow
// rxfifo starts at address 0 in the 1.25kByte FIFO ram, so only
// size needs to be configured
// 32bit data register is used to push to TXfifos and to pop from RXFIFO
// so data access is word sized
static void usbd_configure_rxfifo_size(uint16_t sizeBytes) {
	// 10 words reserved for receiving SETUP packets on control endpoint
	// 1 word for receiving global OUT NAK
	// at least 2 *(largest size packet in words) + 1 word for status written to fifo with each received packet
	uint16_t sizeWords = 10 + 1 + (2 * ((sizeBytes / 4) + 1));
	// Configures the depth of the FIFO.
	MODIFY_REG(USB_OTG_FS->GRXFSIZ,
		USB_OTG_GRXFSIZ_RXFD,
		_VAL2FLD(USB_OTG_GRXFSIZ_RXFD, sizeWords)
		);
	usbd_refresh_fifo_start_addresses();
	}


// Configures the TxFIFO of an IN endpoint.
// TXFIFOs do not have any meta-data associated with the packets, so no need to allocate for that
// endpoint_number The number of the IN endpoint we want to configure its TxFIFO
// sizeBytes The size of the IN endpoint in bytes. (minimum is 16*4, maximum is 512*4)
// However, note that the total fifo RAM itself is only 1.25kBytes
// Any change on any FIFO will update the the registers of all TxFIFOs to adapt the start offsets in the FIFO dedicated memory.

static void usbd_configure_txfifo_size(uint8_t endpoint_number, uint16_t sizeBytes) {
	// Gets the FIFO size in term of 32-bit words.
	uint16_t sizeWords = (sizeBytes + 3) / 4;
	// txfifo control register for EP0 is not array compatible, so needs
	// special handling
	if (endpoint_number == 0)	{
		MODIFY_REG(USB_OTG_FS->DIEPTXF0_HNPTXFSIZ,
			USB_OTG_TX0FD,
			_VAL2FLD(USB_OTG_TX0FD, sizeWords)
			);
		}
	else	{
		MODIFY_REG(USB_OTG_FS->DIEPTXF[endpoint_number - 1],
			USB_OTG_NPTXFD,
			_VAL2FLD(USB_OTG_NPTXFD, sizeWords)
			);
		}
	usbd_refresh_fifo_start_addresses();
	}


// Flushes the RxFIFO of all OUT endpoints.
static void usbd_flush_rxfifo() {
	SET_BIT(USB_OTG_FS->GRSTCTL, USB_OTG_GRSTCTL_RXFFLSH);
	}


// Flushes the TxFIFO of an IN endpoint.
// endpoint_number The number of an IN endpoint to flush its TxFIFO.
static void usbd_flush_txfifo(uint8_t endpoint_number) {
	// Sets the number of the TxFIFO to be flushed and then triggers the flush.
	MODIFY_REG(USB_OTG_FS->GRSTCTL,
		USB_OTG_GRSTCTL_TXFNUM,
		_VAL2FLD(USB_OTG_GRSTCTL_TXFNUM, endpoint_number) | USB_OTG_GRSTCTL_TXFFLSH
		);
	}


// Pops data from the RxFIFO and stores it in the buffer.
// buffer Pointer to the buffer, in which the popped data will be stored.
// sizeBytes Count of bytes to be popped from the dedicated RxFIFO memory.
// reading the memory pops a word from the bottom of the rxfifo regardless
// of where the buffer address is within the 4kbyte address range allocated
// to "Device EP0 /Host Channel 0 Fifo" (see ref manual)
// best to use the starting address in the range allocated to the endpoint 0
// using the FIFO() macro
static void usbd_read_packet(void *pBuffer, uint16_t sizeBytes) {
	// There is only one RxFIFO for all endpoints so just specify 0 as the endpoint.
	// Note : If we write to the address given by FIFO(0) we would be pushing data
	// to the TXFIFO for endpoint 0
	volatile uint32_t *pRxFifo =  FIFO(0);

	while (sizeBytes >= 4)	{
		// Pops one 32-bit word of data (until there is less than one word remaining).
		volatile uint32_t data = *pRxFifo;
		// Stores the data in the buffer.
		*((uint32_t*)pBuffer) = data;
		sizeBytes -= 4;
		pBuffer += 4;
		}

	if (sizeBytes > 0) {
		// Pops the last remaining bytes (which are less than one word).
		volatile uint32_t data = *pRxFifo;
		while (sizeBytes) {
			// Stores the data in the buffer with the correct alignment.
			*((uint8_t*)pBuffer) = data & 0xFF;
			sizeBytes--;
			pBuffer++;
			data >>= 8;
			}
		}
	}


// Pushes a packet into the TxFIFO of an IN endpoint. Note : Does not send the packet,
// just makes it available for reading by the host.
// endpoint_number The number of the endpoint, to which the data will be written.
// buffer Pointer to the buffer contains the data to be written to the endpoint.
// sizeBytes The size of data to be written in bytes.
// write pushes the data to the top of the txfifo, regardless of where the address
// is within the 4kbyte address range allocated to the endpoint
// best to use the first address allocated to the endpoint, using the macro FIFO()
static void usbd_write_packet(uint8_t endpoint_number, const void *buffer, uint16_t sizeBytes) {
	volatile uint32_t *pTxFifo = FIFO(endpoint_number);
	USB_OTG_INEndpointTypeDef *in_endpoint = IN_ENDPOINT(endpoint_number);

	// for simplicity, each transfer has only one packet (PKTCNT = 1) with transfer size = sizeBytes
	MODIFY_REG(in_endpoint->DIEPTSIZ,
		USB_OTG_DIEPTSIZ_PKTCNT | USB_OTG_DIEPTSIZ_XFRSIZ,
		_VAL2FLD(USB_OTG_DIEPTSIZ_PKTCNT, 1) | _VAL2FLD(USB_OTG_DIEPTSIZ_XFRSIZ, sizeBytes)
		);

	// Enables the transmission after clearing both STALL and NAK of the endpoint.
	// Now if device receives any IN tokens it will be ready to send data
	MODIFY_REG(in_endpoint->DIEPCTL,
		USB_OTG_DIEPCTL_STALL,
		USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA
		);

	// Gets the size in term of 32-bit words (to avoid integer overflow in the loop).
	uint16_t sizeWords = (sizeBytes + 3) / 4;
	while (sizeWords) {
		// Pushes the data to the TxFIFO.
		*pTxFifo = *((uint32_t *)buffer);
		sizeWords--;
		buffer += 4;
		}
	}


static void usbd_enumdne_handler() {
	log_info("Speed enumeration done.");
	// maximum packet size for endpoint 0 is 8 bytes
	// this could also be done in rst handler
	usbd_configure_endpoint0(8);
	}

// rxfifo not empty handler
// interrupt is asserted when status bytes and the full data packet have been
// received in the rxfifo
static void usbd_rxflvl_handler() {
	// Pops the status information word from the RxFIFO
	// read the global rxfifo packet status pop register
	// this does a pop operation from the rxfifo so ensure you
	// only read it once !
	// can be abused for reading the data as well ...
	// note that it is possible to get a status word, but no accompanying received data packets
	uint32_t receive_status = USB_OTG_FS_GLOBAL->GRXSTSP;

	// get the endpoint that received the data.
	uint8_t endpoint_number = _FLD2VAL(USB_OTG_GRXSTSP_EPNUM, receive_status);
	// get the count of bytes in the received packet.
	uint16_t pkt_bytes = _FLD2VAL(USB_OTG_GRXSTSP_BCNT, receive_status);
	// get the status of the received packet.
	uint16_t pkt_status = _FLD2VAL(USB_OTG_GRXSTSP_PKTSTS, receive_status);

	switch (pkt_status)	{
	case 0x06:
		log_debug("setup pkt received");
		// SETUP data packet received (includes data).
		// this data packet would have been preceded by a SETUP token packet
		// the response would be dependent on the content of the packet data reeceived
		// e.g. request a specific descriptor, change device address, activate a feature
		// so we first need to pop the data packet and handle it outside
		// the driver layer in the framework layer
    	USB_Events.on_setup_data_received(endpoint_number, pkt_bytes);
    	break;
    case 0x02: // OUT packet (includes data).
		log_debug("OUT pkt received");
    		// endpoint could be control, bulk, interrupt or isochronous
    	// ToDo
		break;
		// above two cases always associated with received data

    case 0x04: // SETUP stage in control transfer has completed.
    	log_debug("setup stage done");
		// after transfer complete on in or out endpoints, core disables
		// endpoint from tx/rx
    	// Re-enables the transmission on the endpoint after every transfer
    	// set the clear NAK bit
        SET_BIT(OUT_ENDPOINT(endpoint_number)->DOEPCTL,
			USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
    	break;

    case 0x03: // OUT data transfer (multiple packets) has completed
    	log_debug("out data xfer done");
    	// note that 0x02 is for a single packet, 0x03 is for the complete transfer
    	// packet less than max size or zero-length packet denotes end
    	// Re-enables the transmission on the endpoint, set the clear NAK bit
        SET_BIT(OUT_ENDPOINT(endpoint_number)->DOEPCTL,
			USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
    	break;
	}
}


static void usbd_set_device_address(uint8_t address) {
    MODIFY_REG(
		USB_OTG_FS_DEVICE->DCFG,
		USB_OTG_DCFG_DAD,
		_VAL2FLD(USB_OTG_DCFG_DAD, address)
		);
	}


// Handles the interrupt raised when an IN endpoint has a raised interrupt.
static void usbd_iepint_handler() {
	// Find the endpoint that caused the interrupt.
	// DAINT bits 0-15 correspond to IN endpoints
	// only one bit should have been set
	// ffs(r) from strings.h returns index of first set least significant bit in r
	// if bit n (n= 0..31) ffs(r) returns n+1, i.e. (1..32)
	// so we need to use ffs(r) - 1
	uint8_t endpoint_number = ffs(USB_OTG_FS_DEVICE->DAINT) - 1;

	// if this IN endpoint raised a transfer completed interrupt
    if (IN_ENDPOINT(endpoint_number)->DIEPINT & USB_OTG_DIEPINT_XFRC)    {
        USB_Events.on_in_transfer_completed(endpoint_number);
        // Clears the interrupt flag.
        SET_BIT(IN_ENDPOINT(endpoint_number)->DIEPINT, USB_OTG_DIEPINT_XFRC);
    	}
	}


// Handles the interrupt raised when an OUT endpoint has a raised interrupt.
static void usbd_oepint_handler(){
	// Find the OUT endpoint that caused the interrupt.
	// bits 16-31 in DAINT correspond to the OUT endpoints
	// only one bit should have been set
	uint8_t endpoint_number = ffs(USB_OTG_FS_DEVICE->DAINT >> 16) - 1;

	// if this OUT endpoint raised a transfer completed interrupt
    if (OUT_ENDPOINT(endpoint_number)->DOEPINT & USB_OTG_DOEPINT_XFRC)    {
        USB_Events.on_out_transfer_completed(endpoint_number);
        // Clears the interrupt;
        SET_BIT(OUT_ENDPOINT(endpoint_number)->DOEPINT, USB_OTG_DOEPINT_XFRC);
    	}
	}



