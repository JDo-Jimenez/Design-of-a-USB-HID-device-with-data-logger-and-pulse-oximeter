#include "stm32f4_usb_hid_device.h"

extern USB_OTG_CORE_HANDLE USB_OTG_dev;
extern USB_HIDDEVICE_Status_t USB_HIDDEVICE_INT_Status;

USB_HIDDEVICE_Status_t USB_HIDDEVICE_Init(void) {
	/* Initialize HID device */
	USBD_Init(&USB_OTG_dev,
	#ifdef USE_USB_OTG_HS 
			USB_OTG_HS_CORE_ID,
	#else            
			USB_OTG_FS_CORE_ID,
	#endif
			&USR_desc, 
			&USBD_HID_cb, 
			&USR_cb);
	
	/* Set not connected */
	USB_HIDDEVICE_INT_Status = USB_HIDDEVICE_Status_Disconnected;
	
	/* Device not connected */
	return USB_HIDDEVICE_INT_Status;
}

USB_HIDDEVICE_Status_t USB_HIDDEVICE_GetStatus(void) {
	/* Return status */
	return USB_HIDDEVICE_INT_Status;
}

/* Keyboard */
USB_HIDDEVICE_Status_t USB_HIDDEVICE_KeyboardStructInit(USB_HIDDEVICE_Keyboard_t* Keyboard_Data) {
	/* Set defaults */
	Keyboard_Data->L_CTRL = USB_HIDDEVICE_Button_Released;
	Keyboard_Data->L_ALT = USB_HIDDEVICE_Button_Released;
	Keyboard_Data->L_SHIFT = USB_HIDDEVICE_Button_Released;
	Keyboard_Data->L_GUI = USB_HIDDEVICE_Button_Released;
	Keyboard_Data->R_CTRL = USB_HIDDEVICE_Button_Released;
	Keyboard_Data->R_ALT = USB_HIDDEVICE_Button_Released;
	Keyboard_Data->R_SHIFT = USB_HIDDEVICE_Button_Released;
	Keyboard_Data->R_GUI = USB_HIDDEVICE_Button_Released;
	Keyboard_Data->Key1 = 0;
	Keyboard_Data->Key2 = 0;
	Keyboard_Data->Key3 = 0;
	Keyboard_Data->Key4 = 0;
	Keyboard_Data->Key5 = 0;
	Keyboard_Data->Key6 = 0;
	
	/* Return currect status */
	return USB_HIDDEVICE_INT_Status;
}

USB_HIDDEVICE_Status_t USB_HIDDEVICE_KeyboardSend(USB_HIDDEVICE_Keyboard_t* Keyboard_Data) {
	uint8_t buff[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};; /* 9 bytes long report */
	
	/* Check status */
	if (USB_HIDDEVICE_INT_Status != USB_HIDDEVICE_Status_Connected) {
		return USB_HIDDEVICE_Status_Disconnected;
	}
	
	/* Report ID */
	buff[0] = 0x01; /* Keyboard */
	
	/* Control buttons */
	buff[1] = 0;
	buff[1] |= Keyboard_Data->L_CTRL 	<< 0;	/* Bit 0 */
	buff[1] |= Keyboard_Data->L_SHIFT 	<< 1;	/* Bit 1 */
	buff[1] |= Keyboard_Data->L_ALT 	<< 2;	/* Bit 2 */
	buff[1] |= Keyboard_Data->L_GUI 	<< 3;	/* Bit 3 */
	buff[1] |= Keyboard_Data->R_CTRL 	<< 4;	/* Bit 4 */
	buff[1] |= Keyboard_Data->R_SHIFT 	<< 5;	/* Bit 5 */
	buff[1] |= Keyboard_Data->R_ALT 	<< 6;	/* Bit 6 */
	buff[1] |= Keyboard_Data->R_GUI 	<< 7;	/* Bit 7 */
	
	/* Padding */
	buff[2] = 0x00;
	
	/* Keys */
	buff[3] = Keyboard_Data->Key1;
	buff[4] = Keyboard_Data->Key2;
	buff[5] = Keyboard_Data->Key3;
	buff[6] = Keyboard_Data->Key4;
	buff[7] = Keyboard_Data->Key5;
	buff[8] = Keyboard_Data->Key6;
	
	/* Send to USB */
	USBD_HID_SendReport(&USB_OTG_dev, buff, 9);
	
	/* Return connected */
	return USB_HIDDEVICE_Status_Connected;	
}

USB_HIDDEVICE_Status_t USB_HIDDEVICE_KeyboardReleaseAll(void) {
	uint8_t buff[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0}; /* 9 bytes long report */
	
	/* Check status */
	if (USB_HIDDEVICE_INT_Status != USB_HIDDEVICE_Status_Connected) {
		return USB_HIDDEVICE_Status_Disconnected;
	}	
	
	/* Report ID */
	buff[0] = 0x01; /* Keyboard */
	
	/* Send to USB */
	USBD_HID_SendReport(&USB_OTG_dev, buff, 9);
	
	/* Return connected */
	return USB_HIDDEVICE_Status_Connected;
}

/* Custom report */
USB_HIDDEVICE_Status_t USB_HIDDEVICE_SendCustom(uint8_t* buff, uint8_t count) {
	/* Check status */
	if (USB_HIDDEVICE_INT_Status != USB_HIDDEVICE_Status_Connected) {
		return USB_HIDDEVICE_Status_Disconnected;
	}	
	
	/* Send to USB */
	USBD_HID_SendReport(&USB_OTG_dev, buff, count);
	
	/* Return connected */
	return USB_HIDDEVICE_Status_Connected;
}
