"""me500emu - an instruction-level emulator of the Mimaki ME-500 engraving machine controller.

Runs the unmodified controller firmware (NEC V33, 512 KiB EPROM) on Unicorn, with models of the
panel, LCD, serial port, interrupt controller, timers, NVRAM and the sub-CPU axis drive.
The firmware image is not included; see roms/README.md.
"""
__version__ = "0.1.0"
