# Bootloader V0.1 contains generic USB VID/PID while attempting to get assigned PID. 

# Build of bootloader was accomplished with the uf2-samdx1 repo and WSL on Windows 11. https://github.com/microsoft/uf2-samdx1.

# Programming of the bootloader was accoplished via OpenOCD and a STM32 ST-Link programmer using PowerShell and Putty.




# Bootloader notes: 
# Modified for SAMD21x17 lower memory map. 


# Command Notes: 

# .\openocd -f C:\openocd\OpenOCD-20250710-0.12.0\share\openocd\scripts\board\samd21.cfg
# flash write_image erase C:\\openocd\\zlcustom_v0.bin 0x0000
