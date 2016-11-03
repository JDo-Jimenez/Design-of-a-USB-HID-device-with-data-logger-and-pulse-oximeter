Not all the files are included, check at the end of this page.

/*Estructura de archivos y de librerías en el proyecto.*/

Se han incluido en el proyecto todos los archivos de librerías necesarios para realizar un
dispositivo USB HID. Los archivos se han obtenido de las librerías CMSIS,
STM32_USB_Device library y STM32_USB_OTG_Driver, también es necesario incluir los
archivos STM32F4xx RCC, STM32F4xx GPIO, STM32F4xx EXTI ya que hay que configurar
algunos pines para recibir y transmitir datos por USB así como también se hace uso de
interrupciones externas.
A continuación se muestra la estructura de archivos en el proyecto. 

Para una mayor descipcion de lo que se ha realizado en cada archivo consultar el apartado 3.3.3 de la memoria.
http://hdl.handle.net/10251/56704


 Discovery STM32F407D
    * Cmsis
         Core_cm4.h
         Core_cm4_simd.h
         Core_cmFunc.h
         Core_ccmInstr.h
    * User
         Stm32f4_usb_hid_device
         Defines.h
         Main.c
         Usb_bsp.c
         Usb_conf.h
         Usbd_conf.h
         Usbd_desc.c
         Usbd_desc.h
         Usbd_usr.c
    * Usb_Hid_Device
         Usb_bsp.h
         Usb_core.c
         Usb_dcd.c
         Usb_dcd_int.c
         Usb_defines.h
         Usbd_core.c
         Usbd_ioreq.c
         Usbd_req.c
         Usbd_usr.h
         Usbd_hid_core.c
         Usbd_hid_core.h
