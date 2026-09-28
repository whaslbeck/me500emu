"""Tests that need no ROM image."""
import os
import subprocess
import sys

import pytest

from me500emu import paths, rominfo, program, costs, fastcore


def test_rom_lookup_env(tmp_path, monkeypatch):
    f = tmp_path / "x.bin"
    f.write_bytes(b"\xff" * 0x80000)
    monkeypatch.setenv("ME500_ROM", str(f))
    assert paths.rom_path("stock") == str(f)
    assert paths.rom_path(explicit=str(f)) == str(f)
    with pytest.raises(paths.RomNotFound):
        paths.rom_path(explicit=str(tmp_path / "missing.bin"))


def test_rominfo_version_block():
    img = bytearray(b"\xff" * 0x80000)
    tag = b"$VER$ 1.50MAX BUILD 000042 2026-01-01T00:00:00Z$"
    img[0x7F001:0x7F001 + len(tag)] = tag
    assert rominfo.version(bytes(img)) == "1.50MAX BUILD 000042 2026-01-01T00:00:00Z"
    assert rominfo.is_max(bytes(img))
    assert rominfo.version(b"\xff" * 0x80000) is None


def test_package_data_present():
    assert os.path.exists(paths.data_path("symbols.txt"))
    for name in ("iic10_default.bin", "device_example.bin"):
        assert os.path.getsize(paths.data_path("nvram", name)) == 512


def test_detect_language():
    assert program.detect_language("G21 G90\nG0 X1 Y1\nG1 X2 F100\n") == "gcode"
    assert program.detect_language("IN;SP1;PU0,0;PD100,100;") == "hpgl"


def test_reference_hpgl_square():
    r = program.reference("IN;PU2000,2000;PD4000,2000,4000,4000,2000,4000,2000,2000;PU0,0;", "hpgl",
                          (0, 0), (0, 0, 0), 1400)
    xy = [(p[0], p[1]) for p in r.points]
    assert (4000 * program.STEPS_PER_UNIT, 4000 * program.STEPS_PER_UNIT) in xy
    assert abs(r.cut_length_mm - 87.0) < 1e-6      # 80 mm square + the 7 mm plunge to the PD depth


def test_cost_table_small(tmp_path):
    code = bytes([0x90, 0x8B, 0x07, 0x89, 0x07, 0xF7, 0xE3, 0xE4, 0x18])   # nop; mov ax,[bx]; mov [bx],ax; mul bx; in al,18h
    t = costs.build_table(code, str(tmp_path))
    assert len(t) == len(code)
    d = [(x - 256 if x >= 128 else x) for x in t]    # sixteenths of a unit against the 1-unit base
    reg, rd, wr, mul, io = d[0], d[1], d[3], d[5], d[7]
    assert reg < 0 < rd and 0 < wr and rd < mul and 0 < io     # measured: register fastest, mul/div and I/O slow
    assert d[2] == d[4] == d[6] == d[8] == 0                    # only instruction starts carry a cost


def test_c_core_builds():
    so = fastcore.build()
    assert os.path.exists(so)


def test_cli_info_runs():
    r = subprocess.run([sys.executable, "-m", "me500emu", "info"], capture_output=True, text=True,
                       env=dict(os.environ, PYTHONPATH=os.path.join(os.path.dirname(__file__), "..", "src")))
    assert r.returncode == 0
    assert "cache directory" in r.stdout


def test_cli_missing_rom(tmp_path):
    r = subprocess.run([sys.executable, "-m", "me500emu", "run", "x.nc", "--rom", str(tmp_path / "none.bin")],
                       capture_output=True, text=True,
                       env=dict(os.environ, PYTHONPATH=os.path.join(os.path.dirname(__file__), "..", "src")))
    assert r.returncode == 2
    assert "not found" in r.stderr


def _ref(text):
    return program.reference(text, "gcode", (0, 0), (0, 0, 0), 0)


def test_reference_gcode_full_circle():
    import math
    r = _ref("G0 X10 Y10\nG1 Z-0.5 F300\nG2 X10 Y10 I5 J0\nG0 Z2\n")
    assert r.arcs == 1 and not r.notes
    arc = [(x / 200.0, y / 200.0) for x, y, z, c in r.points if z == 100]
    assert all(abs(math.hypot(x - 15, y - 10) - 5) < 0.005 for x, y in arc)
    assert arc[1][1] > 10                         # G2 = clockwise: from 9 o'clock upwards


def test_reference_gcode_helix_and_modal_arc():
    r = _ref("G0 X10 Y0\nG2 X10 Y0 Z-2 I-10\nG1 Z0\nG0 X10 Y0\nG3 X0 Y10 I-10\nX-10 Y0 I0 J-10\n")
    assert r.arcs == 3 and not r.notes
    assert max(p[2] for p in r.points) == 400      # helix ends 2 mm below zero
    assert r.points[-1][:2] == (-2000, 0)


def test_reference_gcode_arc_stops_like_the_branch():
    for text, why in (("G2 X1 Y1 R5\n", "R form"), ("G18\nG2 X1 I1\n", "XY plane"),
                      ("G0 X10 Y0\nG2 X0 Y11 I-10\n", "radius differ"), ("G0 X1\nG2 X1 I-0.005\n", "radius under"),
                      ("G0 X1\nG2 X1 Z-100 I-1\n", "pitch")):
        r = _ref(text)
        assert r.notes and "stop expected" in r.notes[-1] and why in r.notes[-1], (text, r.notes)
