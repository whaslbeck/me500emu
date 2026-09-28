"""Identify a ROM image: stock 1.50 or a 1.50MAX build.

1.50MAX images carry a version block at 0xFF000 (file offset 0x7F000): ``$VER$ 1.50MAX BUILD nnnnnn <time>$``.
The stock image has none. Identify images by this block, not by file name or checksum.
"""
import hashlib
import re

from . import paths

VER_RE = re.compile(rb"\$VER\$ ([ -~]+?)\$")


def version(rom):
    """'1.50MAX BUILD 000019 2026-...' for a 1.50MAX image, '1.50' for the stock image, None for anything else.
    `rom` is the image as bytes or a file path."""
    data = rom if isinstance(rom, (bytes, bytearray)) else open(rom, "rb").read()
    m = VER_RE.search(bytes(data[0x7F000:0x7F100]))
    if m:
        return m.group(1).decode("ascii")
    if hashlib.md5(bytes(data)).hexdigest() == paths.STOCK_ROM_MD5:
        return "1.50"
    return None


def is_max(rom):
    v = version(rom)
    return bool(v and v.startswith("1.50MAX"))
