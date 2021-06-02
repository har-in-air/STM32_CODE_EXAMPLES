#include <stddef.h>
#include "util/logger.h"
#include "util/misc.h"
#include "usb/usb_device.h"
#include "usb/usb_standards.h"
#include "usb/usbd_descriptors.h"
#include "usb/usbd_driver.h"
#include "usb/usbd_framework.h"
#include "hid_report.h"

static void usbf_reset_received_handler();
static void usbf_setup_data_received_handler(uint8_t endpoint_number, uint16_t byte_count);
static void usbf_process_request();
static void usbf_process_standard_device_request();
static void usbf_process_control_transfer_stage();
static void usbf_polled_handler();
static void usbf_in_transfer_completed_handler(uint8_t endpoint_number);
static void usbf_out_transfer_completed_handler(uint8_t endpoint_number);
static void usbf_process_standard_interface_request();
static void usbf_process_class_interface_request();

static USB_DEVICE_t USB_Device;
static uint32_t 	Buffer[8];

// 1. get device descriptor, asks for 64 bytes, though descriptor could be some other size
//   1a. device sends descriptor with 18bytes  (reason for seeing "malformed packet" in wireshark)
//   1b. host reads the first 8bytes of the descriptor response packet, the 8th byte is bMaxPacketSize0
//       which is what the host needs to communicate further with the device. At this point the host
//       ignores the rest of the data, and sends a RESET request
// 2. set device address
//    2a. device sets adddress
// 3. get configuration descriptor
// 3a.   device returns config desc, interface desc,  endpoint desc,  hid desc
// 4. set configuration (configuration index specified in wValue)
// 5. set idle (class and interface specific request eg. HID, tells specific interface to start responding only to interrupt events)
// 6. get hid report request
//   ---- enumeration done ------
// 7 periodic interrupt in request targeting specific endpoint for mouse
//   7a. respond with mousehid report

USB_EVENTS_t USB_Events = {
	.on_usb_reset_received = &usbf_reset_received_handler,
	.on_setup_data_received = &usbf_setup_data_received_handler,
	.on_usb_polled = &usbf_polled_handler,
	.on_in_transfer_completed = &usbf_in_transfer_completed_handler,
	.on_out_transfer_completed = &usbf_out_transfer_completed_handler
	};


void usbf_initialize(){
	USB_Device.ptr_out_buffer = Buffer;
	USB_Driver.initialize_gpio_pins();
	USB_Driver.initialize_core();
	USB_Driver.connect();
	}


void usbf_poll(){
	USB_Driver.poll();
	}


void usbf_configure(){
	// endpoint 0 was configured on initialization (control endpoint)
	// here we configure the endpoints associated with this configuration index
	// not a static function as it is application layer dependent
	// can move it to an application code file later
	USB_Driver.configure_in_endpoint(
		(Config_Desc_Set.mouse_ep_desc.bEndpointAddress & 0x0F), // endpoint #
		(Config_Desc_Set.mouse_ep_desc.bmAttributes & 0x03), // type of endpoint is INTERRUPT
		Config_Desc_Set.mouse_ep_desc.wMaxPacketSize
		);
	// see ref manual, after all endpoints are configured, we need to send a status IN packet
	// to the endpoint
	// i.e. an empty data packet
	USB_Driver.write_packet(
		(Config_Desc_Set.mouse_ep_desc.bEndpointAddress & 0x0F),
		NULL,
		0
		);
	}


static void usbf_reset_received_handler(){
	USB_Device.in_data_size = 0;
	USB_Device.out_data_size = 0;
	USB_Device.config_index = 0;
	USB_Device.device_state = USB_DEVICE_STATE_DEFAULT;
	USB_Device.control_transfer_stage = USB_CONTROL_STAGE_SETUP; // wait for setup token
	USB_Driver.set_device_address(0); // default power-on device address = 0
	}


static void usbf_setup_data_received_handler(uint8_t endpoint_number, uint16_t byte_count){
	// setup data => data is for control OUT endpoint  0
	// pop the data from the rxfifo using the driver function
	USB_Driver.read_packet(USB_Device.ptr_out_buffer, byte_count);
	log_debug_array("SETUP data: ", USB_Device.ptr_out_buffer, byte_count);
	usbf_process_request();
	}


// usb request (setup transaction data) structure is standard
static void usbf_process_request(){
	const USB_REQUEST_t  *pRequest = (const USB_REQUEST_t *)USB_Device.ptr_out_buffer;

	switch(pRequest->bmRequestType & (USB_BM_REQUEST_TYPE_MASK | USB_BM_REQUEST_TYPE_RECIPIENT_MASK))	{
		// standard request targeted at device
		case USB_BM_REQUEST_TYPE_STANDARD | USB_BM_REQUEST_TYPE_RECIPIENT_DEVICE:
			usbf_process_standard_device_request();
		break;
		
		// e.g. set idle request, this is class-specific request and targets a specific interface
		case USB_BM_REQUEST_TYPE_CLASS | USB_BM_REQUEST_TYPE_RECIPIENT_INTERFACE:
			usbf_process_class_interface_request();
		break;
		
		// standard request but targeted to specific interface
		case USB_BM_REQUEST_TYPE_STANDARD | USB_BM_REQUEST_TYPE_RECIPIENT_INTERFACE:
			usbf_process_standard_interface_request();
		break;
		
		default :
		log_error("unknown request");
		break;		
		}
	}


static void usbf_process_standard_device_request(){
	const USB_REQUEST_t * pRequest = (const USB_REQUEST_t *)USB_Device.ptr_out_buffer;

	switch(pRequest->bRequest)	{
		case USB_STANDARD_GET_DESCRIPTOR:
			log_info("Std_get_desc");
			const uint8_t descriptor_type = pRequest->wValue >> 8;
			const uint16_t descriptor_length = pRequest->wLength;
			//const uint8_t config_index = pRequest->wValue & 0xff;

			switch(descriptor_type)	{
			case USB_DESCRIPTOR_TYPE_DEVICE:
				log_info("-get_device_desc");
				USB_Device.ptr_in_buffer = &Device_Descriptor;
				USB_Device.in_data_size = descriptor_length;

				log_info("ctrl xfer stage -> IN-DATA.");
				USB_Device.control_transfer_stage = USB_CONTROL_STAGE_DATA_IN;
				break;

			case USB_DESCRIPTOR_TYPE_CONFIGURATION:
				log_info("-get_config_desc");
				// send configuration descriptor + descriptors for the attached interfaces and endpoints
				// if there is more than one device configuration, the requested configuration index
				// is specified in the wValue LS byte
				USB_Device.ptr_in_buffer = &Config_Desc_Set;
				USB_Device.in_data_size = descriptor_length;
				log_info("ctrl xfer stage -> IN-DATA.");
				USB_Device.control_transfer_stage = USB_CONTROL_STAGE_DATA_IN;
				break;

			default:
				break;
				}
			break;
			
		case USB_STANDARD_SET_ADDRESS:
			// Normally, the new USB device address must be set only after the completion of the transaction
			// (after the status stage) that transmitted the SET ADDRESS request. Otherwise, the sent/received packets
			// will have the new address, while they should maintain the address 0 to finish the current transaction.
			// STM32F4 microcontrollers allow changing the address to the new address even before completing
			// the transaction. The PHY will take care of maintaining the address 0 till the end of the current transaction.
			log_info("Std_set_addr");
			// no data stage required for this OUT transfer because wValue contains the set address argument
			const uint16_t device_address = pRequest->wValue;
			USB_Driver.set_device_address(device_address);
			USB_Device.device_state = USB_DEVICE_STATE_ADDRESSED;
			log_info("ctrl xfer stage -> IN-STATUS.");
			USB_Device.control_transfer_stage = USB_CONTROL_STAGE_STATUS_IN;
			break;
			
		case USB_STANDARD_SET_CONFIG:
			// after host gets configuration descriptor, host decides which configuration
			// the device should use
			log_info("Std_set_config");
			USB_Device.config_index = pRequest->wValue;
		    usbf_configure();
			USB_Device.device_state = USB_DEVICE_STATE_CONFIGURED;
			// no data stage required as the config index was passed in wValue
			log_info("ctrl xfer stage -> IN-STATUS.");
			USB_Device.control_transfer_stage = USB_CONTROL_STAGE_STATUS_IN;
			break;
			
		default :
			log_error("uknown request");
			break;
		}		
	}


static void usbf_process_class_interface_request(){
	const USB_REQUEST_t * pRequest = (const USB_REQUEST_t *)USB_Device.ptr_out_buffer;

	switch(pRequest->bRequest)	{
		case USB_HID_SETIDLE:
		    log_info("HID_SETIDLE ctrl xfer -> IN-STATUS.");
			USB_Device.control_transfer_stage = USB_CONTROL_STAGE_STATUS_IN;
			break;
		default:
			log_error("? class interface request");
			break;		
		}
	}


static void usbf_process_standard_interface_request(){
	const USB_REQUEST_t * pRequest = (const USB_REQUEST_t *)USB_Device.ptr_out_buffer;

	switch (pRequest->wValue >> 8)	{
		case USB_DESCRIPTOR_TYPE_HID_REPORT:
			USB_Device.ptr_in_buffer = &HID_Report_Descriptor;
			USB_Device.in_data_size = HID_Report_Desc_Size_Bytes;
			//log_info("HID_Report_Desc  %d bytes", HID_Report_Desc_Size_Bytes);
			log_info("ctrl xfer ->IN-STATUS.");
			USB_Device.control_transfer_stage = USB_CONTROL_STAGE_DATA_IN;
			break;

		default :
			log_error("? std interface request");
			break;
		}
	}


// usb protocol knows that a transfer is complete when
// 1. the sent/received data packet is smaller than the max packet size for the endpoint.
// 2. sent/received data packet has 0 data bytes. this is required when completing data
// packet size = max packet size

static void usbf_process_control_transfer_stage(){
	switch(USB_Device.control_transfer_stage)	{
		case USB_CONTROL_STAGE_SETUP:
			// work already done by setup_data_received_handler
			break;
		case USB_CONTROL_STAGE_DATA_IN:
			log_info("Processing IN-DATA stage.");
			// in our case we have only one control IN endpoint 0
			// if remaining data to send is greater than max packet size
			// send max packet size, else send remaining data
			uint8_t data_size = MIN(USB_Device.in_data_size, Device_Descriptor.bMaxPacketSize0);

		    USB_Driver.write_packet(0, USB_Device.ptr_in_buffer, data_size);
		    USB_Device.in_data_size -= data_size;
		    USB_Device.ptr_in_buffer += data_size;

			// need to go into a different "sub-stage" to avoid overwriting the data before it
			// has been read by the host. This is not in the usb spec, just our framework
			// implementation
			log_info("ctrl stage -> IN-DATA IDLE.");
		    USB_Device.control_transfer_stage = USB_CONTROL_STAGE_DATA_IN_IDLE;

			// finished multi-packet data transfer
		    if (USB_Device.in_data_size == 0)  {
				// last data packet size == max packet size, so we need to
				// send a packet with 0 bytes so that host knows the transfer is complete		    
		    	if (data_size == Device_Descriptor.bMaxPacketSize0)	{
		    		log_info("ctrl stage -> IN-DATA ZERO.");
		    		USB_Device.control_transfer_stage = USB_CONTROL_STAGE_DATA_IN_ZERO;
		    		}
		    	else {
					// last data packet size was less than max packet size, so host
					// knows that transfer is complete, will acknowledge  status		    	
		    		log_info("ctrl stage -> OUT-STATUS.");
		    		USB_Device.control_transfer_stage = USB_CONTROL_STAGE_STATUS_OUT;
		    		}
		    	}
			break;
		case USB_CONTROL_STAGE_DATA_IN_IDLE:
			// when the in transfer is completed, the event handler
			// in_transfer_completed_handler is called, it changes
			// USB_Device.control_transfer_stage  to USB_CONTROL_STAGE_DATA_IN		
			break;
		case USB_CONTROL_STAGE_STATUS_OUT:
			// host will send a zero length packet to acknowledge the received data
			// there is nothing to be done, just switch back to setup stage		
			log_info("ctrl stage -> SETUP.");
			USB_Device.control_transfer_stage = USB_CONTROL_STAGE_SETUP;
			break;
		case USB_CONTROL_STAGE_STATUS_IN:
			// acknowledge with a zero-length packet on control endpoint 0 txfifo
			USB_Driver.write_packet(0, NULL, 0);
			log_info("ctrl xfer stage -> SETUP.");
			USB_Device.control_transfer_stage = USB_CONTROL_STAGE_SETUP;
			break;
		case USB_CONTROL_STAGE_DATA_OUT:
			break;
		case USB_CONTROL_STAGE_DATA_IN_ZERO:
			// see usbf_in_transfer_completed_handler, the action is done
			// there		
			break;
		}
	}


// called each time the interrupts/status register bits are checked
static void usbf_polled_handler(){
	usbf_process_control_transfer_stage();
	}


// we call it transfer and not transaction, because it is possible to
// program the registers to send multiple transactions in one data write,
// but this implementation for simplicity uses only one transaction
// this handler is called after the usb host has completed reading a
// data packet from the IN endpoint
static void usbf_in_transfer_completed_handler(uint8_t endpoint_number){
	// still some data to read, go back to DATA_IN stage
	if (USB_Device.in_data_size)	{
		log_info("ctrl stage -> IN-DATA.");
		USB_Device.control_transfer_stage = USB_CONTROL_STAGE_DATA_IN;
		}
	else 
	if (USB_Device.control_transfer_stage == USB_CONTROL_STAGE_DATA_IN_ZERO)	{
		// note that the control transfer stage switch statement does not execute
		// any actions in USB_CONTROL_STAGE_DATA_IN_ZERO, its done here

		// write 0-length data packet to tell usb host that the transfer is complete, 
		// then switch to STATUS OUT to be ready for the USB host to acknowledge 
		// with a 0-length packet
		USB_Driver.write_packet(0, NULL, 0);
		log_info("ctrl stage -> OUT-STATUS.");
		USB_Device.control_transfer_stage = USB_CONTROL_STAGE_STATUS_OUT;
		}

	// after standard acknowledgement for IN Transfer has been done
	// we respond to  URB_INTERRUPT_IN request targeting hid mouse endpoint
	if (endpoint_number == (Config_Desc_Set.mouse_ep_desc.bEndpointAddress & 0x0F))	{
		hid_write_mouse_report();
		}
	}


static void usbf_out_transfer_completed_handler(uint8_t endpoint_number){
	}



