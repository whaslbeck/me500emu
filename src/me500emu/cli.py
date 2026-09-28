"""Command line: `me500emu <command>` (or `python -m me500emu <command>`).

    me500emu info                       ROM images found, their versions, cache directory
    me500emu boot [--rom R]             cold-boot once and cache the snapshot (later starts take a second)
    me500emu run JOB [--rom R] [--mode M] [--report out.json] [--trace out.csv]
                                        run a job headless and report time, bounds, final position, status
    me500emu bridge [--rom R] ...       the machine as a serial device on a pty
    me500emu ui [--port 8000] ...       browser UI (3D path view, LCD, panel, job loader)
"""
import argparse
import json
import os
import sys


def cmd_info(argv):
    from . import paths, rominfo, __version__
    print("me500emu %s" % __version__)
    print("cache directory: %s" % paths.cache_dir())
    for which in ("stock", "max"):
        env, name = paths.ROM_FILES[which]
        try:
            p = paths.rom_path(which)
            print("%-5s ROM: %s  (%s)" % (which, p, rominfo.version(p) or "unrecognised image"))
        except paths.RomNotFound:
            print("%-5s ROM: not found (set %s or put %s into %s)" % (which, env, name, paths.ROMS_DIR))
    return 0


def cmd_boot(argv):
    ap = argparse.ArgumentParser(prog="me500emu boot", description="Cold-boot a ROM once and cache the snapshot.")
    ap.add_argument("--rom", default="stock")
    ap.add_argument("--nvram", default=None, help="NVRAM seed (default: data/nvram/device_example.bin)")
    ap.add_argument("--rebuild", action="store_true", help="boot again even if a snapshot exists")
    a = ap.parse_args(argv)
    import time
    from . import paths, rominfo
    from . import snapshot_state as snap
    from .session import DEFAULT_NVRAM
    rom = paths.rom_path(a.rom)
    t = time.time()
    m = snap.booted(rom, physics=True, trace="fast", nvram_seed=a.nvram or paths.data_path("nvram", DEFAULT_NVRAM),
                    rebuild=a.rebuild)
    print("%s (%s) booted in %.1f s, LCD: %s" % (rom, rominfo.version(rom), time.time() - t,
                                                 " | ".join(r.rstrip() for r in m.panel.text())))
    return 0


def cmd_run(argv):
    ap = argparse.ArgumentParser(prog="me500emu run", description="Run a job file headless and report the result.")
    ap.add_argument("job", help="HP-GL or G-code file")
    ap.add_argument("--rom", default="stock", help="stock | max | FILE")
    ap.add_argument("--mode", default=None, choices=["std", "opt", "gcode"],
                    help="default: gcode for .nc/.gcode/.ngc/.tap files on a 1.50MAX image, opt for other files on "
                         "a 1.50MAX image, std on the stock image")
    ap.add_argument("--baud", type=int, default=9600, choices=[9600, 19200, 38400])
    ap.add_argument("--handshake", default="hard", choices=["hard", "code"])
    ap.add_argument("--nvram", default=None)
    ap.add_argument("--z0", type=float, default=None, help="Z0 in mm below the top of the Z travel (G-code default 10)")
    ap.add_argument("--timeout", type=float, default=3600.0, help="emulated seconds before giving up")
    ap.add_argument("--report", default=None, help="write the JSON report here (default: stdout)")
    ap.add_argument("--trace", default=None, help="write the travelled path as CSV (t_s,x,y,z)")
    a = ap.parse_args(argv)
    from . import paths, rominfo
    from .session import Session, SessionError
    from .jobs import run_job
    rom = paths.rom_path(a.rom)
    mode = a.mode
    if mode is None:
        if rominfo.is_max(rom):
            mode = "gcode" if os.path.splitext(a.job)[1].lower() in (".nc", ".gcode", ".ngc", ".tap", ".g") else "opt"
        else:
            mode = "std"
    text = open(a.job, "rb").read()
    try:
        s = Session(rom=rom, mode=mode, baud=a.baud, handshake=a.handshake, nvram=a.nvram, z0_mm=a.z0)
    except SessionError as e:
        print("error: %s" % e, file=sys.stderr)
        return 2

    def progress(sent, total, instr, _last=[0]):
        if sys.stderr.isatty() and instr - _last[0] >= 20000000:
            _last[0] = instr
            sys.stderr.write("\r  %d/%d bytes, %.1f s machine time " % (sent, total, instr / 2e6))
    r = run_job(s, text, timeout_s=a.timeout, keep_points=bool(a.trace), progress=progress)
    if sys.stderr.isatty():
        sys.stderr.write("\n")
    r["job"] = os.path.abspath(a.job)
    if a.trace:
        with open(a.trace, "w") as fh:
            fh.write("t_s,x_mm,y_mm,z_mm\n")
            for p in r.pop("points"):
                fh.write("%.4f,%.4f,%.4f,%.4f\n" % p)
    out = json.dumps(r, indent=1)
    if a.report:
        open(a.report, "w").write(out + "\n")
        print("report: %s  (machine time %.2f s, %s)" % (a.report, r["machine_time_s"],
                                                       "TIMEOUT" if r["timed_out"] else "halted" if r["status"]["halted"] else "ok"))
    else:
        print(out)
    st = r["status"]
    bad = r["timed_out"] or st["halted"] or st["uart_overruns"] or any(
        st.get(k) for k in ("stop", "error", "overflow", "line_overflow", "dropped"))
    return 1 if bad else 0


def cmd_bridge(argv):
    from . import bridge
    bridge.main(argv)
    return 0


def cmd_ui(argv):
    from .ui import server
    server.main(argv)
    return 0


COMMANDS = {"info": cmd_info, "boot": cmd_boot, "run": cmd_run, "bridge": cmd_bridge, "ui": cmd_ui}


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or argv[0] in ("-h", "--help") or argv[0] not in COMMANDS:
        print(__doc__)
        return 0 if argv and argv[0] in ("-h", "--help") else 2
    try:
        return COMMANDS[argv[0]](argv[1:])
    except Exception as e:
        from .paths import RomNotFound
        if isinstance(e, RomNotFound):
            print("error: %s" % e, file=sys.stderr)
            return 2
        raise


if __name__ == "__main__":
    sys.exit(main())
