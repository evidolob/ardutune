#!/usr/bin/env python3
"""Download ArduPilot's Copter parameter metadata and write the compact copy
ArduTune bundles (crates/core/data/copter-params.json.gz), so parameter
descriptions work offline.

    python3 tools/update-param-docs.py            # latest (master)
    python3 tools/update-param-docs.py FILE.json  # from a downloaded apm.pdef.json
"""

import gzip
import json
import sys
import urllib.request
from datetime import date
from pathlib import Path

URL = "https://autotest.ardupilot.org/Parameters/ArduCopter/apm.pdef.json"
OUT = Path(__file__).resolve().parent.parent / "crates/core/data/copter-params.json.gz"


def number(text):
    try:
        return float(text)
    except (TypeError, ValueError):
        return None


def compact(doc):
    out = {"name": doc.get("DisplayName", ""), "desc": " ".join(doc.get("Description", "").split())}
    if doc.get("Units"):
        out["units"] = doc["Units"]
    rng = doc.get("Range") or {}
    lo, hi = number(rng.get("low")), number(rng.get("high"))
    if lo is not None and hi is not None:
        out["range"] = [lo, hi]
    if doc.get("Increment") and number(doc["Increment"]) is not None:
        out["step"] = number(doc["Increment"])
    if doc.get("Values"):
        values = [[number(k), v] for k, v in doc["Values"].items() if number(k) is not None]
        out["values"] = sorted(values)
    if doc.get("Bitmask"):
        bits = [[int(k), v] for k, v in doc["Bitmask"].items() if k.isdigit()]
        out["bits"] = sorted(bits)
    if doc.get("RebootRequired") == "True":
        out["reboot"] = True
    if doc.get("ReadOnly") == "True":
        out["readonly"] = True
    if doc.get("User") == "Advanced":
        out["advanced"] = True
    return out


def main():
    if len(sys.argv) > 1:
        raw = json.loads(Path(sys.argv[1]).read_text())
    else:
        with urllib.request.urlopen(URL, timeout=60) as r:
            raw = json.load(r)
    params = {}
    for group in raw.values():
        if not isinstance(group, dict):
            continue
        for name, doc in group.items():
            if isinstance(doc, dict) and name not in params:
                params[name] = compact(doc)
    data = {"source": URL, "fetched": date.today().isoformat(), "params": dict(sorted(params.items()))}
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_bytes(gzip.compress(json.dumps(data, separators=(",", ":")).encode(), mtime=0))
    print(f"{len(params)} parameters -> {OUT} ({OUT.stat().st_size / 1e3:.0f} kB)")


if __name__ == "__main__":
    main()
