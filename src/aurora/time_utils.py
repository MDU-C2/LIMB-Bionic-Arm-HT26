# Majd
"""Shared UTC timestamps for saved samples and metadata."""

from datetime import datetime, timezone


def utc_now() -> str:
    """Return an ISO timestamp for saved samples and metadata."""
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")
