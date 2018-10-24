-------------------------------------------------------------------------------------------------------------------------------------
                        STEVAL-WeSU1 - Internal flash initialisation and Restore all the FACTORY Firmware
-------------------------------------------------------------------------------------------------------------------------------------						
To initialise the STEVAL-WeSU1 board, just double-click on STEVAL-WESU1_FACTORY.bat batch file.

Erase First Flash Sector (address 0x08000000)

Program EEPROM: Application (address 0x08080FF0) 		with -> SetAppAddress.hex
Program Flash APPLICATION (address 0x08020000)			with -> WeSU.hex
Program Usb BlueNRG-MS Bridge (address 0x08008000)		with -> WESU_BlueNRG_VCOM_1_8.hex
Program Flash OTA SERVICE MANAGER (address 0x08003800)		with -> WeSU_OTA_ServiceManager_App.hex
Program Flash RESET MANAGER (address 0x08003000)		with -> ResetManager.hex
Program EEPROM Delete Registers					with -> ResetRegs.hex
Program Flash DFU bootloader (address 0x08000000)		with -> Wesu2DFU.hex

Finally, an MCU Reset command is issued to start the board

-------------------------------------------------------------------------------------------------------------------------------------
                        STEVAL-WeSU1 - Program Flash APPLICATION - DEMONSTRATION firmware

-------------------------------------------------------------------------------------------------------------------------------------
To update the Demonstration Application STEVAL-WeSU1 board, just double-click on ProgApplication.bat batch file.

It loads the binary files ( OTA bootloader and application files) necessary to properly configure the board for normal operation.

Program Flash APPLICATION (address 0x08020000)			with -> WeSU_demo.hex

Finally, an MCU Reset command is issued to start the board

-------------------------------------------------------------------------------------------------------------------------------------
                        STEVAL-WeSU1 - Program Flash APPLICATION - EXAMPLES firmware

-------------------------------------------------------------------------------------------------------------------------------------
To update the Examples Application STEVAL-WeSU1 board, just double-click on ProgExamples.bat batch file.

It loads the binary files ( OTA bootloader and application files) necessary to properly configure the board for normal operation.

Program Flash APPLICATION (address 0x08020000)			with -> WeSU_examples.hex

Finally, an MCU Reset command is issued to start the board

-------------------------------------------------------------------------------------------------------------------------------------
                        STEVAL-WeSU1 - Program EEPROM: Application address on EEPROM

-------------------------------------------------------------------------------------------------------------------------------------
To Set Application addr on EEPROM, just double-click on SetAppAddress.bat batch file.

Program EEPROM: Application (address 0x08080FF0) 		with -> SetAppAddress.hex

Finally, an MCU Reset command is issued to start the board

-------------------------------------------------------------------------------------------------------------------------------------
                        STEVAL-WeSU1 - Program EEPROM: Delete registers

-------------------------------------------------------------------------------------------------------------------------------------
To restore the default values just double-click on ResetRegs.bat batch file.

Program EEPROM Delete Registers					with -> ResetRegs.hex

Finally, an MCU Reset command is issued to start the board

-------------------------------------------------------------------------------------------------------------------------------------
                        STEVAL-WeSU1 - Program EEPROM: Delete Licenses

-------------------------------------------------------------------------------------------------------------------------------------
To delete the licenses just double-click on ResetLics.bat batch file.

Program EEPROM Delete Licenses					with -> ResetLics.hex

Finally, an MCU Reset command is issued to start the board

-------------------------------------------------------------------------------------------------------------------------------------
                        STEVAL-WeSU1 - Program Flash OTA SERVICE MANAGER

-------------------------------------------------------------------------------------------------------------------------------------
To update the OTA Manager, double-click on WeSU_OTA_ServiceManager_App.bat batch file.

Program Flash OTA SERVICE MANAGER (address 0x08003800)		with -> WeSU_OTA_ServiceManager_App.hex

-------------------------------------------------------------------------------------------------------------------------------------
                        STEVAL-WeSU1 - Program Flash Ota 1.2 Updater

-------------------------------------------------------------------------------------------------------------------------------------
To update the OTA Manager to ver 1.2, double-click on ProgOta1.2Updater.bat batch file.

Program Flash OTA SERVICE MANAGER (address 0x08060000)		with -> WeSU_Upg_OTA12.hex

-------------------------------------------------------------------------------------------------------------------------------------
                        STEVAL-WeSU1 - Program Flash BlueNRG Updater

-------------------------------------------------------------------------------------------------------------------------------------
To upgrade the BlueNRG-MS chip to version 7.2a, double-click on ProgBluenrgUpdater.bat batch file.

Program Flash OTA SERVICE MANAGER (address 0x08060000)		with -> WESU_BlueNRG_Stack_Upd_7_2a.hex

-------------------------------------------------------------------------------------------------------------------------------------
                        STEVAL-WeSU1 - Program Usb BlueNRG-MS Bridge

-------------------------------------------------------------------------------------------------------------------------------------
To update the BlueNRG Bridge, double-click on SetBluenrgUsbBridgeAddress.bat batch file.

Program Usb BlueNRG-MS Bridge (address 0x08008000)		with -> WESU_BlueNRG_VCOM_1_8.hex

Finally, an MCU Reset command is issued to start the board

-------------------------------------------------------------------------------------------------------------------------------------
                        STEVAL-WeSU1 - Program Flash DFU bootloader

-------------------------------------------------------------------------------------------------------------------------------------
To update the DFU Manager, just double-click on SetUsbDfuAddress.bat batch file.

Program Flash DFU bootloader (address 0x08000000)		with -> Wesu2DFU.hex

Finally, an MCU Reset command is issued to start the board