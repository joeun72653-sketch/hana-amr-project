#!/usr/bin/env python3
"""Minimal safe handling of duplicate WebSocket STOP requests in AMR normal bridge.

Changes ONLY ~/amr_remote_client/amr_ws_to_ros_normal.py.
Preserves legitimate DISARM on operator request, loss of connection, timeout, faults.
Existing CAN, Depth, WebSocket server and Lift files remain unchanged.
"""
from pathlib import Path
from datetime import datetime
import ast
import os
import tempfile

p = Path.home() / 'amr_remote_client' / 'amr_ws_to_ros_normal.py'
if not p.is_file():
    raise SystemExit(f'[ERROR] File not found: {p}')
s = p.read_text(encoding='utf-8')
marker = '# NORMAL_ARM_STOP_GUARD_V1'
if marker in s:
    print('[SKIP] Fix already applied; no changes made.')
    raise SystemExit(0)

old = '''    def stop(self, disarm=False):
        state = self.state()
        with self.lock:
            previous_phase = self.phase
'''
new = '''    def stop(self, disarm=False):
        state = self.state()
        with self.lock:
            # NORMAL_ARM_STOP_GUARD_V1
            # Repeated key-release STOPs must not reset the ongoing
            # physical stop verification window. Genuine DISARM is unaffected.
            if not disarm and self.phase == 'STOPPING':
                return
            previous_phase = self.phase
'''
if s.count(old) != 1:
    raise SystemExit(f'[ERROR] stop() pattern mismatch ({s.count(old)} matches); unchanged.')
s = s.replace(old,new,1)

# Preserve existing motor-ready / stop checks. Do not weaken safety conditions.
# Add trace messages for distinct real DISARM causes to make diagnosis faster.
needle = '''            if disarm:
                self.session = None
                self.phase = 'DISARMED'
                self.auth_pub.publish(Bool(data=False))'''
replacement = '''            if disarm:
                # Log the disarm source rather than silently suppressing it.
                print(f'[DISARM TRACE] prior_phase={previous_phase} '
                      f'fault_reason={self.fault_reason!r} '
                      f'session_active={self.session is not None}', flush=True)
                self.session = None
                self.phase = 'DISARMED'
                self.auth_pub.publish(Bool(data=False))'''
if s.count(needle) != 1:
    raise SystemExit(f'[ERROR] DISARM trace pattern mismatch ({s.count(needle)}); unchanged.')
s = s.replace(needle,replacement,1)

try:
    ast.parse(s, filename=str(p))
except SyntaxError as e:
    raise SystemExit(f'[ERROR] Python syntax check failed: {e}')
backup = p.with_name(p.name+'.backup_armfix_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
backup.write_bytes(p.read_bytes())
fd, tmp = tempfile.mkstemp(prefix=p.name+'.', suffix='.tmp', dir=str(p.parent))
try:
    with os.fdopen(fd,'w',encoding='utf-8') as f:
        f.write(s)
    os.chmod(tmp,p.stat().st_mode)
    os.replace(tmp,p)
except BaseException:
    if os.path.exists(tmp): os.unlink(tmp)
    raise
print('[BACKUP]', backup)
print('[OK]', p)
print('[CHECK] Duplicate STOP no longer resets STOPPING timer; genuine DISARM remains enabled.')
print('[NOTE] This does not guarantee ARM stays on; [DISARM TRACE] helps diagnose remaining causes.')
