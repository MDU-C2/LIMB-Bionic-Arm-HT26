"""LIMB manual CAN panel. Starts in simulation; live mode requires verified firmware.
Python 3 + Tkinter; python-can only required for live SocketCAN on Jetson.
"""
import argparse
import math
import struct
import time
import tkinter as tk
from tkinter import ttk, messagebox

# CAN actuation payload: absolute joint angle [deg], velocity [deg/s], <ff.
AXES = {
    'elbow': ('Bicep / Elbow', 0x240, 0x140, 0x4A0),
    'updown': ('Shoulder up/down', 0x220, 0x120, 0x4A2),
    'leftright': ('Shoulder left/right', 0x221, 0x121, 0x4A3),
    'upper': ('Upper arm rotation', 0x222, 0x122, 0x4A1),
}

MAX_ABS_TARGET_DEG = 10.0
MAX_STEP_DEG = 5.0


def payload(target, speed):
    if not math.isfinite(target) or not math.isfinite(speed) or not 0 < speed <= 1:
        raise ValueError('Finite target and speed >0 to 1 deg/s required.')
    return struct.pack('<ff', target, speed)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--live', action='store_true')
    p.add_argument('--channel', default='can0')
    p.add_argument('--verified', nargs='+', choices=['elbow', 'upper', 'updown', 'leftright'], default=[])
    a = p.parse_args()
    if a.live and not a.verified:
        p.error('Live mode requires --verified with bench-validated axes.')
    root = tk.Tk()
    root.title('LIMB - Manual CAN Motor Test')
    root.resizable(False, False)
    bus = None
    can = None
    if a.live:
        if not messagebox.askyesno('Bench safety', 'Confirm isolated test bench, encoder-free TEST firmware, unloaded joints, independent power cutoff, and no person attached to the robot.\n\nProceed with live CAN?'):
            root.destroy(); return
        try:
            import can
            bus = can.Bus(interface='socketcan', channel=a.channel, bitrate=1_000_000)
        except Exception as exc:
            messagebox.showerror('CAN unavailable', str(exc)); root.destroy(); return
    ttk.Label(root, text='LIVE CAN - ENCODER-FREE TEST' if bus else 'DEMO - no CAN transmission', font=('TkDefaultFont', 13, 'bold')).grid(row=0, column=0, columnspan=5, pady=12)
    for col, title in enumerate(('Motor', 'Estimated (deg)', 'Target (deg)', 'Speed (deg/s)', 'Stop')):
        ttk.Label(root, text=title).grid(row=1, column=col, padx=8)
    fields, readings = {}, {}
    status = tk.StringVar(value='Select motors. Targets are relative to the boot position (0 deg).')
    log = tk.Text(root, width=86, height=9, state='disabled')
    active = set()
    stop_timer = None
    def write(s):
        log.configure(state='normal'); log.insert('end', s+'\n'); log.see('end'); log.configure(state='disabled')
    def send(ident, data):
        if bus is not None:
            bus.send(can.Message(arbitration_id=ident, data=data, is_extended_id=False), timeout=0.1)
        write(('TX submitted' if bus else 'PREVIEW') + f' 0x{ident:03X}  {data.hex(" ")}')
    def stop(keys):
        failed = []
        for key in keys:
            try:
                send(AXES[key][2], b''); active.discard(key)
            except Exception as exc:
                failed.append(key); write(f'STOP TX FAILED {key}: {exc}')
        status.set('STOP requested; not confirmed.' if not failed else 'STOP failed: use physical power cutoff!')

    def auto_stop():
        nonlocal stop_timer
        stop_timer = None
        if active:
            stop(list(active))
    for row, (key, (label, _, _, _)) in enumerate(AXES.items(), 2):
        selected = tk.BooleanVar(value=False)
        current = tk.StringVar(value='--' if bus else '0.0 (demo)')
        target = tk.StringVar(value='3.0')
        speed = tk.StringVar(value='1.0')
        cb = ttk.Checkbutton(root, text=label, variable=selected)
        cb.grid(row=row, column=0, sticky='w', padx=8, pady=5)
        if bus and key not in a.verified:
            cb.state(['disabled'])
        ttk.Label(root, textvariable=current).grid(row=row, column=1)
        ttk.Entry(root, textvariable=target, width=12).grid(row=row, column=2, padx=8)
        ttk.Entry(root, textvariable=speed, width=12).grid(row=row, column=3, padx=8)
        ttk.Button(root, text='Stop', command=lambda k=key: stop([k])).grid(row=row, column=4, padx=8)
        fields[key] = selected, current, target, speed
    def start():
        nonlocal stop_timer
        if active:
            messagebox.showwarning('Active test', 'Stop the current test before starting another.'); return
        commands = []
        try:
            for key, (sel, _, tgt, vel) in fields.items():
                if not sel.get(): continue
                target, speed = float(tgt.get()), float(vel.get())
                data = payload(target, speed)
                if bus:
                    if key not in a.verified: raise ValueError(f'{key}: not verified.')
                    measured, stamp = readings.get(key, (math.nan, 0))
                    if not math.isfinite(measured) or time.monotonic()-stamp > 1.5:
                        raise ValueError(f'{key}: missing fresh joint-angle feedback.')
                    if abs(target-measured) > MAX_STEP_DEG:
                        raise ValueError(f'{key}: target must be within {MAX_STEP_DEG:g} degrees of feedback.')
                if abs(target) > MAX_ABS_TARGET_DEG:
                    raise ValueError(f'{key}: target must stay within +/-{MAX_ABS_TARGET_DEG:g} degrees of boot zero.')
                commands.append((key, data))
            if not commands: raise ValueError('Select at least one motor.')
        except ValueError as exc:
            messagebox.showerror('Invalid test', str(exc)); return
        try:
            for key, data in commands:
                active.add(key)
                send(AXES[key][1], data)
        except Exception as exc:
            write(f'TX failed: {exc}'); stop(list(active)); return
        status.set('Commands submitted (motion not confirmed).' if bus else 'Demo: commands previewed.')
        if stop_timer is not None: root.after_cancel(stop_timer)
        stop_timer = root.after(7000, auto_stop)
    def poll():
        if bus:
            try:
                for _ in range(100):
                    msg = bus.recv(timeout=0)
                    if msg is None: break
                    if msg.is_extended_id or msg.is_remote_frame or msg.is_error_frame or len(msg.data)!=4: continue
                    for key, (_, _, _, report) in AXES.items():
                        if msg.arbitration_id==report:
                            value, = struct.unpack('<f', msg.data)
                            if math.isfinite(value):
                                readings[key] = value, time.monotonic()
                                fields[key][1].set(f'{value:.2f}')
                for key, (_, current, _, _) in fields.items():
                    if key in readings and time.monotonic()-readings[key][1]>1.5:
                        current.set('STALE')
            except Exception as exc:
                write(f'RX error: {exc}'); stop(list(active))
        root.after(100, poll)
    ttk.Button(root, text='START SELECTED', command=start).grid(row=6, column=0, columnspan=2, sticky='ew', padx=8, pady=8)
    ttk.Button(root, text='STOP ALL', command=lambda: stop(AXES)).grid(row=6, column=2, columnspan=3, sticky='ew', padx=8, pady=8)
    ttk.Label(root, textvariable=status, wraplength=600).grid(row=7, column=0, columnspan=5, pady=6)
    log.grid(row=8, column=0, columnspan=5, padx=8, pady=8)
    def close():
        if bus:
            stop(AXES); bus.shutdown()
        root.destroy()
    root.protocol('WM_DELETE_WINDOW', close)
    root.after(100, poll)
    root.mainloop()


if __name__ == '__main__':
    main()
