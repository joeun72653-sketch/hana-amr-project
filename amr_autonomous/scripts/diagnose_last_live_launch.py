#!/usr/bin/env python3
"""Print the first relevant failure in the last supervised AMR console log.

The final SIGTERM/-15 lines are usually a *consequence* of Launch shutdown;
diagnose earlier lifecycle, TF, camera, CAN or map failures first.
Read-only, no ROS subscriptions or motor commands.
"""
from pathlib import Path
import re
import sys

ROOT = Path.home()/".ros"/"amr_live_console"
CRITICAL = re.compile(
    r"\[LIFECYCLE ERROR\]|\[LOCALIZATION FAILED\]|"
    r"\[NAV2 STARTUP FAILED\]|\[AMR\].*(FAILED|will NOT|shutdown)|"
    r"(?i:traceback|exception|fatal error|permission denied|address already in use|"
    r"node.*(died|crashed)|process has died|timed out waiting for transform|"
    r"motor state missing|port.*busy|cannot publish a pose)")
CONSEQUENCE = re.compile(
    r"exit code -15|sending signal 'SIG(INT|TERM)'|"
    r"process\[.*\] failed to terminate|process has died.*exit code -15"
)

def main():
    files = sorted(ROOT.glob("amr_live_*.log"), key=lambda p: p.stat().st_mtime, reverse=True)
    if not files:
        print("[DIAG] No saved live console log found. New run_live.sh saves to "+str(ROOT))
        return 1
    path=files[0]
    print("[DIAG] Last live log: "+str(path))
    lines=path.read_text(errors="replace").splitlines()
    hits=[(i,line) for i,line in enumerate(lines) if CRITICAL.search(line) and
          not CONSEQUENCE.search(line)]
    if not hits:
        print("[DIAG] No obvious first cause detected. Showing the last 50 lines.")
        print("\n".join(lines[-50:]))
        return 0
    print(f"[DIAG] {len(hits)} relevant warning/error lines (first 30):")
    for i,line in hits[:30]:
        print(f"  [{i+1:05d}] {line[:450]}")
    idx=hits[0][0]
    print("[DIAG] Context around first relevant issue:")
    for i in range(max(0,idx-8),min(len(lines),idx+9)):
        print(f"  [{i+1:05d}] {lines[i][:450]}")
    return 0

if __name__=="__main__":
    sys.exit(main())
