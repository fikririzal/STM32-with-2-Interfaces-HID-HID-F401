# STM32-with-2-Interfaces-HID-HID-F401
Documentation of the progress of my project from RapidTrigger OSUpad by adding a HID with 3 Axis joystick.

The Device Descriptor and Configuration Descriptor can be understood by the HOST, the Keyboard and Joystick are registered.

The final problem with this code is that when sending a report with Endpoint `0x82`, no data is visible on USBPcap, but if I just send To Endpoint `0x81` (Keyborad HID report) the data is visible in USBPcap.

[The USBPcap file](https://github.com/fikririzal/STM32-with-2-Interfaces-HID-HID-F401/blob/51666a001dcdec496e83d72473536c88d66549a2/USBPcap%20File/STM32-with-2-Interfaces-HID-HID-F401.pcapng)

One of myRefence USBPcap file is my gaming mouse, with 3 interfaces (Mouse, Keyboard, Vendor)

[Mouse USBPcap file](https://github.com/fikririzal/STM32-with-2-Interfaces-HID-HID-F401/blob/1cafba283af2a12412ec40e187a9b92fc48d6459/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c#L1936-L1940)





## Changed File
Changed File related to Problem above after Code Generation
### stm32f4xx_hal_pcd.c
```
  if (ep_addr == 0x82) {
	ep = &hpcd->IN_ep[0x81 & EP_ADDR_MSK];
  } else {
	ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];
  }
```
[To codeline](https://github.com/fikririzal/STM32-with-2-Interfaces-HID-HID-F401/blob/1cafba283af2a12412ec40e187a9b92fc48d6459/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c#L1936-L1940), I think this change is because the index `ep_addr` array if `0x82` contains an empty config endpoint, therefore I switched it to index `0x81` which contains the default config endpoint
### usbd_customhid.c

```
uint8_t USBD_CUSTOM_HID_SendReport(USBD_HandleTypeDef *pdev,
                                   uint8_t *report, uint16_t len, uint8_t Ep)
{
```
[To codeline](https://github.com/fikririzal/STM32-with-2-Interfaces-HID-HID-F401/blob/1cafba283af2a12412ec40e187a9b92fc48d6459/Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Src/usbd_customhid.c#L691-L693), I gave the function a new parameter, namely `uint8_t Ep` so that I can choose which endpoint the `uint8_t *report` packet is sent to
### usbd_customhid.c

```
uint8_t USBD_CUSTOM_HID_SendReport(USBD_HandleTypeDef *pdev,
                                   uint8_t *report, uint16_t len, uint8_t Ep)
{
```
[To codeline](https://github.com/fikririzal/STM32-with-2-Interfaces-HID-HID-F401/blob/1cafba283af2a12412ec40e187a9b92fc48d6459/Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Src/usbd_customhid.c#L691-L693), I gave the function a new parameter, namely `uint8_t Ep` so that I can choose which endpoint the `uint8_t *report` packet is sent to