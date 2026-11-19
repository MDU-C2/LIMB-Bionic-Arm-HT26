"""Worker-process adapters used by the desktop export actions."""

from __future__ import annotations

import argparse
from pathlib import Path

from aurora.recording.common import enable_ctrl_break_as_interrupt
from aurora.recording.exports import export_dmp, export_recording


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("bundle", "dmp"))
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--preferred", choices=("clean", "raw"), default="clean")
    parser.add_argument("--saved", action="store_true")
    args = parser.parse_args(argv)
    enable_ctrl_break_as_interrupt(stdin_stop=True)
    try:
        if args.action == "bundle":
            output = export_recording(
                args.source,
                args.output,
                progress=lambda done, total: print(f"Export: {done}/{total} files", flush=True),
            )
        else:
            print("Fitting/exporting trajectory...", flush=True)
            output = export_dmp(
                args.source, args.output, preferred=args.preferred, refit=not args.saved
            )
        print(f"Saved {output}", flush=True)
        return 0
    except (KeyboardInterrupt, InterruptedError):
        print("Export cancelled. Original data is unchanged.", flush=True)
        return 130
    except (OSError, ValueError, KeyError) as error:
        print(f"Export failed: {error}", flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
