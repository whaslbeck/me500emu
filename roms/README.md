# ROM images

The emulator runs the machine's real firmware. **The firmware is copyrighted by Mimaki and is not part of this
repository.** You need your own image, read from your own machine.

## Stock firmware 1.50

The ME-500 controller carries one 40-pin EPROM labelled `ME-500 C-0MV1.50` (MN27C4096 / 27C4096, 256 K x 16 = 512 KiB).
It is socketed. Read it with any programmer that handles 27C4096 (e.g. a TL866II+/T48 with the 27C4096 profile) and
save it as a flat 512 KiB little-endian image, low byte first, exactly as the CPU sees it.

Put it here as `roms/mimaki-me500-c-0mv1.50.bin`, or point `ME500_ROM` at it.

Expected checksums of an unmodified 1.50 image:

    MD5     bc21fcc4f50d8d79456f19ddaf7b7680
    SHA256  5baaac69fbc4feedaba5e24ee12b52e1246d3ad6f3834b79f21c093ddb3ecd18

`me500emu info` tells you whether the file was recognised. If your checksum differs, check the byte order first
(swapped bytes are the usual mistake) - a different firmware revision has not been seen yet and is untested.

## 1.50MAX images (optional)

1.50MAX is an extended firmware built on top of 1.50 (look-ahead path planning, G-code mode with arcs, remote
commands). If you have an image, put it here as `roms/me500_max.bin` or point `ME500_ROM_MAX` at it; the emulator
recognises it by its version block (`$VER$ 1.50MAX BUILD ...` at 0xFF000), not by the file name.

## Lookup order

1. an explicit path (`--rom FILE`)
2. `ME500_ROM` / `ME500_ROM_MAX`
3. this directory: `mimaki-me500-c-0mv1.50.bin` / `me500_max.bin`
4. `./roms/` below the current working directory

**Never commit ROM images.** `.gitignore` excludes everything in this directory except this file.
