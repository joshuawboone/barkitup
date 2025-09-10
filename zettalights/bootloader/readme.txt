# Bootloader V0.1 contains generic USB VID/PID while attempting to get assigned PID. 

# Build of bootloader was accomplished with the uf2-samdx1 repo and WSL on Windows 11. https://github.com/microsoft/uf2-samdx1.

# Programming of the bootloader was accoplished via OpenOCD and a STM32 ST-Link programmer using PowerShell and Putty.




# Bootloader notes: 
# Modified for SAMD21x17 lower memory map. 


# Command Notes: 

# .\openocd -f C:\openocd\OpenOCD-20250710-0.12.0\share\openocd\scripts\board\samd21.cfg
# flash write_image erase C:\\openocd\\zlcustom_v0.bin 0x0000


# Arduino IDE notes:

# Getting the board definition working involved modifications to the existing Adafruit board definitions by adding a custom board. 

# Included full modified boards.txt and required variant folder(s) to select the board in Arduino IDE. %localappdata%\Arduino15\packages\adafruit\hardware\samd\1.7.16\

# Currently only supports UF2 upload. Export Sketch to BIN, convert to UF2 via python script, drag and drop to board in bootloader mode. 
# python uf2conv.py -c -f 0x68ed2b88 -b 0x2000 sketch_sep7a.ino.bin -o test2.uf2


# Testing to get the board supported directly in IDE requires modifications to the base BOSSAC version and ran into numerous issues. Still testing. 
