"""Golden scenario for the regression test: a fixed HP-GL job on the stock firmware, from the cached session.

Prints instruction count, strobes, a RAM hash and the job report as JSON. Run it in a fresh process each time
(the C core is a process singleton). `python tests/golden_run.py > tests/golden_stock.json` regenerates the
reference - only do that for an intended behaviour change, and say why in the commit.
"""
import hashlib
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from me500emu.session import Session  # noqa: E402
from me500emu.jobs import run_job  # noqa: E402

JOB = "PU2000,2000;PD4000,2000,4000,4000,2000,4000,2000,2000;PU0,0;"

s = Session(rom="stock", mode="std")
r = run_job(s, JOB)
ram = bytes(s.u.mem_read(0, 0x10000))
out = dict(instr=s.m.instr - s.instr0, strobes=s.status()["strobes"], ram_sha1=hashlib.sha1(ram).hexdigest(),
           machine_time_s=r["machine_time_s"], bounds_mm=r["bounds_mm"], cut_bounds_mm=r["cut_bounds_mm"],
           final_mm=r["final_mm"], lcd=r["lcd"])
print(json.dumps(out, indent=1, sort_keys=True))
