set STLINK_PATH="C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\"
@echo off
color 0F
echo.
echo                /**********************************************/
echo                /***                                        ***/
echo                /***     Program WESU SYS configuration     ***/
echo                /***                                        ***/
echo                /**********************************************/
echo.
echo.
echo                /**********************************************/
echo                  Program EEPROM: Delete registers
echo                /**********************************************/
%STLINK_PATH%ST-LINK_CLI.exe -c SWD UR -P "ResetRegs.hex"

echo Reset MCU
%STLINK_PATH%ST-LINK_CLI.exe -Rst

if NOT "%1" == "SILENT" pause
