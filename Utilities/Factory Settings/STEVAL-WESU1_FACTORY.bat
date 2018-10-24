set STLINK_PATH="C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\"
@echo off
color 0F
echo.
echo                /**********************************************/
echo                /***                                        ***/
echo                /***   Program WESUL_v2 SYS configuration   ***/
echo                /***                                        ***/
echo                /**********************************************/
echo.
echo.


echo                /******************************************/
echo                          Erase First Flash Sector
echo                            (address 0x08000000)
echo                                Step 1 of 9
echo                /******************************************/
%STLINK_PATH%ST-LINK_CLI.exe -c SWD UR -SE 0

echo                /******************************************/
echo                 Program EEPROM: Application addr on EEPROM
echo                            (address 0x08080FF0)
echo                                Step 2 of 9
echo                /******************************************/
%STLINK_PATH%ST-LINK_CLI.exe -c SWD UR -P "SetAppAddress.hex"

echo                /******************************************/
echo                         Program Flash APPLICATION
echo                            (address 0x08020000)
echo                                Step 3 of 9
echo                /******************************************/
%STLINK_PATH%ST-LINK_CLI.exe -c SWD UR -P "WeSU_demo.hex" -V "after_programming"

echo                /******************************************/
echo                         Program Usb BlueNRG-MS Bridge
echo                             (address 0x08008000)
echo                                Step 4 of 9
echo                /******************************************/
%STLINK_PATH%ST-LINK_CLI.exe -c SWD UR -P "WESU_BlueNRG_VCOM_1_8.hex" -V "after_programming"

echo                /******************************************/
echo                         Program Flash OTA SERVICE MANAGER
echo                             (address 0x08003800)
echo                                Step 5 of 9
echo                /******************************************/
%STLINK_PATH%ST-LINK_CLI.exe -c SWD UR -P "WeSU_OTA_ServiceManager_App.hex" -V "after_programming"

echo                /******************************************/
echo                         Program Flash RESET MANAGER
echo                             (address 0x08003000)
echo                                Step 6 of 9
echo                /******************************************/
%STLINK_PATH%ST-LINK_CLI.exe -c SWD UR -P "ResetManager.hex" -V "after_programming"

echo                /**********************************************/
echo                  Program EEPROM: Delete registers
echo                                Step 7 of 9
echo                /**********************************************/
%STLINK_PATH%ST-LINK_CLI.exe -c SWD UR -P "ResetRegs.hex"

echo                /******************************************/
echo                         Program Flash DFU bootloader
echo                             (address 0x08000000)
echo                                Step 8 of 9
echo                /******************************************/
%STLINK_PATH%ST-LINK_CLI.exe -c SWD UR -P "Wesu2DFU.hex" -V "after_programming"

echo                /******************************************/
echo                                 RESET MCU
echo                                Step 9 of 9
echo                /******************************************/
echo MCU Reset 
%STLINK_PATH%ST-LINK_CLI.exe -Rst


if NOT "%1" == "SILENT" pause
