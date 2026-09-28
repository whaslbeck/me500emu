"""Where the emulator finds ROM images, package data and its cache.

The Mimaki firmware is not part of this project. You supply the image(s) yourself - see `roms/README.md`.

ROM lookup, first match wins:

1. an explicit path passed by the caller (``--rom FILE``);
2. the environment: ``ME500_ROM`` (stock 1.50), ``ME500_ROM_MAX`` (a 1.50MAX image, optional);
3. ``<project>/roms/`` next to ``src/``: ``mimaki-me500-c-0mv1.50.bin`` (stock) and ``me500_max.bin`` (1.50MAX).

Cache (boot snapshots, cost tables, the UI's NVRAM file, the compiled C core): ``ME500EMU_CACHE`` or
``~/.cache/me500emu``.
"""
import os

PACKAGE_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(PACKAGE_DIR, "data")
PROJECT_DIR = os.path.dirname(os.path.dirname(PACKAGE_DIR))       # .../mimaki_me500_emulator (source checkout)
ROMS_DIR = os.path.join(PROJECT_DIR, "roms")

ROM_FILES = {"stock": ("ME500_ROM", "mimaki-me500-c-0mv1.50.bin"),
             "max": ("ME500_ROM_MAX", "me500_max.bin")}

# MD5 of the stock image as read from an ME-500 EPROM labelled `ME-500 C-0MV1.50` (27C4096, 512 KiB).
STOCK_ROM_MD5 = "bc21fcc4f50d8d79456f19ddaf7b7680"


class RomNotFound(FileNotFoundError):
    pass


def rom_path(which="stock", explicit=None):
    """Path of a ROM image. `which` is "stock", "max" or a file name/path."""
    if explicit:
        if not os.path.exists(explicit):
            raise RomNotFound("ROM image not found: %s" % explicit)
        return explicit
    if which not in ROM_FILES:
        return rom_path(explicit=which)
    env, name = ROM_FILES[which]
    candidates = [os.environ.get(env), os.path.join(ROMS_DIR, name), os.path.join(os.getcwd(), "roms", name)]
    for c in candidates:
        if c and os.path.exists(c):
            return c
    raise RomNotFound("no %s ROM image: set %s or put %s into %s (see roms/README.md)" % (which, env, name, ROMS_DIR))


def data_path(*parts):
    return os.path.join(DATA_DIR, *parts)


def cache_dir():
    d = os.environ.get("ME500EMU_CACHE") or os.path.join(os.path.expanduser("~"), ".cache", "me500emu")
    os.makedirs(d, exist_ok=True)
    return d


def cache_path(name):
    return os.path.join(cache_dir(), name)
