"""LIMB encoderless CAN panel. Starts in preview mode.
Python 3 and Tkinter. Live SocketCAN also requires python-can.
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

POLL_MS = 100
STATUS_TIMEOUT = 0.5
START_ANGLE = 0.0  # All four firmware demo references.
ARM = struct.pack('<ff', START_ANGLE, -0.0)
STOP = struct.pack('<ff', START_ANGLE, 0.0)


def payload(target, speed):
    if not math.isfinite(target) or not math.isfinite(speed) or not 0 < speed <= 1:
        raise ValueError('Finite target and speed >0 to 1 deg/s required.')
    return struct.pack('<ff', target, speed)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--live', action='store_true')
    p.add_argument('--channel', default='can0')
    p.add_argument('--verified', nargs='+', choices=AXES, default=[],
                   help='Axes explicitly bench-validated with encoderless firmware. '
                        'Include elbow only after validating its STEP count, direction and travel.')
    a = p.parse_args()
    if a.live and not a.verified:
        p.error('Live mode requires --verified with bench-validated axes.')
    root = tk.Tk()
    root.title('LIMB - Manual CAN Motor Test')
    root.resizable(False, False)
    bus = None
    can = None
    if a.live:
        if not messagebox.askyesno(
                'Bench safety',
                'Confirm encoderless demo firmware, bench validation for every listed axis, '
                'an isolated bench, an independent power cutoff, and nobody attached to the robot. '
                'Elbow validation must cover its STEP count, direction, and travel.'
                '\n\nProceed with live CAN?'):
            root.destroy()
            return
        try:
            import can
            bus = can.Bus(interface='socketcan', channel=a.channel, bitrate=1_000_000)
        except Exception as exc:
            messagebox.showerror('CAN unavailable', str(exc)); root.destroy(); return
    ttk.Label(root, text='LIVE CAN - isolated bench' if bus else 'DEMO - no CAN transmission', font=('TkDefaultFont', 13, 'bold')).grid(row=0, column=0, columnspan=5, pady=12)
    for col, title in enumerate(('Motor', 'Estimated (deg)', 'Target (deg)', 'Speed (deg/s)', 'Stop')):
        ttk.Label(root, text=title).grid(row=1, column=col, padx=8)
    fields, readings = {}, {}
    status = tk.StringVar(value='Select motors and confirm the start pose before motion.' if bus
                          else 'Select motors and enter absolute joint targets.')
    log = tk.Text(root, width=86, height=9, state='disabled')
    active = {}
    armed, arming = set(), {}
    stop_timer = None
    def write(s):
        log.configure(state='normal'); log.insert('end', s+'\n'); log.see('end'); log.configure(state='disabled')
    def send(ident, data, *, log_frame=True):
        if bus is not None:
            bus.send(can.Message(arbitration_id=ident, data=data, is_extended_id=False), timeout=0.1)
        if log_frame:
            write(('TX submitted' if bus else 'PREVIEW') + f' 0x{ident:03X}  {data.hex(" ")}')

    def fresh(key):
        return key in readings and time.monotonic() - readings[key][1] <= STATUS_TIMEOUT

    def stop(keys, *, emergency=False):
        keys = list(keys)
        failed = []
        for key in keys:
            active.pop(key, None)
            if emergency:
                armed.discard(key)
                arming.pop(key, None)
            try:
                send(AXES[key][2 if emergency else 1], b'' if emergency else STOP)
            except Exception as exc:
                failed.append(key)
                write(f'Stop TX failed for {key}: {exc}')
        if failed and not emergency:
            stop(failed, emergency=True)
        elif failed:
            status.set('Emergency stop TX failed. Use physical power cutoff.')
        elif emergency:
            status.set('Emergency stop requested. Reset and realign affected axes before arming.')
        elif keys:
            status.set('Normal stop requested. Estimates remain available after completed bursts.')

    def arm():
        keys = [key for key, (sel, _, _, _) in fields.items() if sel.get() and key not in armed]
        if not keys or active or arming:
            return
        if not messagebox.askyesno(
                'Confirm start pose',
                'Confirm each selected joint is physically at its configured 0° reference '
                'and has stayed there since power-up. See the demo README for the joint references.'):
            return
        if any(key not in a.verified or not fresh(key) or
               not math.isnan(readings[key][0]) for key in keys):
            status.set('Arming requires a fresh unavailable estimate from each reset demo axis.')
            return
        try:
            for key in keys:
                arming[key] = time.monotonic()
                send(AXES[key][1], ARM)
        except Exception as exc:
            write(f'Arming TX failed: {exc}')
            stop(arming, emergency=True)
            return
        status.set('Waiting for the demo firmware to report the starting estimate.')

    for row, (key, (label, _, _, _)) in enumerate(AXES.items(), 2):
        selected = tk.BooleanVar(value=False)
        current = tk.StringVar(value='--' if bus else '0.0 (demo)')
        target = tk.StringVar(value='3.0' if not bus else '')
        speed = tk.StringVar(value='1.0')
        cb = ttk.Checkbutton(root, text=label, variable=selected)
        cb.grid(row=row, column=0, sticky='w', padx=8, pady=5)
        if bus and key not in a.verified:
            cb.state(['disabled'])
        ttk.Label(root, textvariable=current).grid(row=row, column=1)
        ttk.Entry(root, textvariable=target, width=12).grid(row=row, column=2, padx=8)
        ttk.Entry(root, textvariable=speed, width=12).grid(row=row, column=3, padx=8)
        ttk.Button(root, text='Stop', command=lambda k=key: stop([k]) if k in active else None).grid(row=row, column=4, padx=8)
        fields[key] = selected, current, target, speed
    def start():
        nonlocal stop_timer
        if active or arming:
            messagebox.showwarning('Test in progress', 'Finish arming or stop the current test before starting.')
            return
        commands = []
        try:
            for key, (sel, _, tgt, vel) in fields.items():
                if not sel.get(): continue
                target, speed = float(tgt.get()), float(vel.get())
                if bus:
                    if key not in a.verified or key not in armed:
                        raise ValueError(f'{key}: bench verification and start-pose confirmation required.')
                    if not fresh(key) or not math.isfinite(readings[key][0]):
                        raise ValueError(f'{key}: missing fresh position estimate.')
                    if abs(target - readings[key][0]) > 5:
                        raise ValueError(f'{key}: target must be within 5 degrees of the estimated position.')
                elif abs(target) > 5:
                    raise ValueError('Demo bench limit: target within +/-5 degrees of demo zero.')
                commands.append((key, payload(target, speed)))
            if not commands: raise ValueError('Select at least one motor.')
        except ValueError as exc:
            messagebox.showerror('Invalid test', str(exc)); return
        try:
            for key, data in commands:
                active[key] = data  # Stop even if the send result is uncertain.
                send(AXES[key][1], data)
        except Exception as exc:
            write(f'TX failed: {exc}')
            stop(active, emergency=True)
            return
        status.set('Commands submitted (motion not confirmed).' if bus else 'Demo: commands previewed.')
        if stop_timer is not None: root.after_cancel(stop_timer)
        stop_timer = root.after(7000, lambda: stop(list(active)) if active else None)
    def poll():
        nonlocal poll_timer
        if bus:
            try:
                for _ in range(100):
                    msg = bus.recv(timeout=0)
                    if msg is None:
                        break
                    if msg.is_extended_id or msg.is_remote_frame or msg.is_error_frame or len(msg.data) != 4:
                        continue
                    age = max(0.0, time.time() - msg.timestamp)
                    if age > STATUS_TIMEOUT:
                        continue
                    for key, (_, _, _, report) in AXES.items():
                        if msg.arbitration_id == report:
                            value, = struct.unpack('<f', msg.data)
                            readings[key] = value, time.monotonic() - age
                            break
                for key, (_, current, _, _) in fields.items():
                    value, stamp = readings.get(key, (math.nan, 0))
                    if key in arming:
                        if stamp >= arming[key] and value == START_ANGLE and fresh(key):
                            armed.add(key)
                            del arming[key]
                            if not arming:
                                status.set('Start pose confirmed. Select a target and start.')
                        elif time.monotonic() - arming[key] > STATUS_TIMEOUT:
                            write(f'{key}: arming unconfirmed')
                            stop([key], emergency=True)
                    if not fresh(key) or not math.isfinite(value):
                        armed.discard(key)
                    if key in armed:
                        current.set(f'{value:.2f}')
                    elif key in readings:
                        current.set('UNAVAILABLE' if fresh(key) else 'STALE')
                if active.keys() - armed:
                    write('Position estimate lost during motion.')
                    stop(active, emergency=True)
                for key, data in active.items():
                    send(AXES[key][1], data, log_frame=False)
            except Exception as exc:
                write(f'CAN I/O error: {exc}')
                stop(active, emergency=True)
        poll_timer = root.after(POLL_MS, poll)

    if bus:
        ttk.Button(root, text='CONFIRM START POSE', command=arm).grid(
            row=6, column=0, columnspan=5, sticky='ew', padx=8, pady=4)
    ttk.Button(root, text='START SELECTED', command=start).grid(row=7, column=0, columnspan=2, sticky='ew', padx=8, pady=8)
    ttk.Button(root, text='STOP ALL', command=lambda: stop(active)).grid(row=7, column=2, sticky='ew', padx=8, pady=8)
    ttk.Button(root, text='EMERGENCY STOP', command=lambda: stop(AXES, emergency=True)).grid(
        row=7, column=3, columnspan=2, sticky='ew', padx=8, pady=8)
    ttk.Label(root, textvariable=status, wraplength=600).grid(row=8, column=0, columnspan=5, pady=6)
    log.grid(row=9, column=0, columnspan=5, padx=8, pady=8)
    def close():
        root.after_cancel(poll_timer)
        if stop_timer is not None:
            root.after_cancel(stop_timer)
        try:
            if bus:
                stop(active)
                bus.shutdown()
        finally:
            root.destroy()

    root.protocol('WM_DELETE_WINDOW', close)
    poll_timer = root.after(POLL_MS, poll)
    root.mainloop()


if __name__ == '__main__':
    main()
