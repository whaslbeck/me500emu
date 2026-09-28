"""Tests that boot the firmware. The first run builds the snapshots (~2 min), later runs take seconds."""
import hashlib
import json
import os
import subprocess
import sys

import pytest

from me500emu import paths

HERE = os.path.dirname(os.path.abspath(__file__))
EXAMPLES = os.path.join(HERE, "..", "examples")
ENV = dict(os.environ, PYTHONPATH=os.path.join(HERE, "..", "src"))

pytestmark = pytest.mark.slow


def _golden():
    r = subprocess.run([sys.executable, os.path.join(HERE, "golden_run.py")], capture_output=True, text=True, env=ENV)
    assert r.returncode == 0, r.stderr
    return json.loads(r.stdout)


def test_golden_stock(rom_stock):
    """Bit-identical behaviour on the stock firmware: instruction count, strobes and RAM after a fixed job."""
    if hashlib.md5(open(rom_stock, "rb").read()).hexdigest() != paths.STOCK_ROM_MD5:
        pytest.skip("stock ROM checksum differs from the reference image")
    got = _golden()                             # identical whether or not the snapshots had to be built first
    want = json.load(open(os.path.join(HERE, "golden_stock.json")))
    assert got == want


def _run(rom, job, *extra):
    r = subprocess.run([sys.executable, "-m", "me500emu", "run", job, "--rom", rom] + list(extra),
                       capture_output=True, text=True, env=ENV)
    assert r.returncode == 0, r.stdout + r.stderr
    return json.loads(r.stdout)


def test_hpgl_square_stock(rom_stock):
    r = _run(rom_stock, os.path.join(EXAMPLES, "square.hpgl"))
    cb = r["cut_bounds_mm"]
    assert abs(cb["x"][1] - 40.0) < 0.01 and abs(cb["y"][1] - 40.0) < 0.01
    assert abs(cb["z"][0] + 7.0) < 0.01                  # ZD 7.00 on the panel of the example NVRAM
    assert abs(r["final_mm"]["x"]) < 0.01 and abs(r["final_mm"]["y"]) < 0.01
    assert not r["timed_out"] and r["status"]["halted"] == 0
    assert r["lcd"][0].startswith("[REMOTE]")
    assert 1.0 < r["machine_time_s"] < 20.0


def test_gcode_circle_max(rom_max):
    r = _run(rom_max, os.path.join(EXAMPLES, "circle.nc"))
    cb = r["cut_bounds_mm"]
    assert r["mode"] == "gcode"
    assert r["status"]["stop"] == 0 and r["status"]["error"] == 0
    for axis in ("x", "y"):
        lo, hi = cb[axis]
        assert abs((hi - lo) - 10.0) < 0.05, (axis, lo, hi)   # circle r = 5 mm
    assert abs(r["final_mm"]["x"]) < 0.01 and abs(r["final_mm"]["y"]) < 0.01
