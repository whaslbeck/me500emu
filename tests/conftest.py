import os
import sys

import pytest

SRC = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src")
if SRC not in sys.path:
    sys.path.insert(0, SRC)

from me500emu import paths  # noqa: E402


def pytest_configure(config):
    config.addinivalue_line("markers", "slow: boots the firmware (first run ~2 min, then cached snapshots)")


def _rom(which):
    try:
        return paths.rom_path(which)
    except paths.RomNotFound:
        return None


@pytest.fixture(scope="session")
def rom_stock():
    p = _rom("stock")
    if p is None:
        pytest.skip("no stock ROM image (see roms/README.md)")
    return p


@pytest.fixture(scope="session")
def rom_max():
    p = _rom("max")
    if p is None:
        pytest.skip("no 1.50MAX image")
    return p
