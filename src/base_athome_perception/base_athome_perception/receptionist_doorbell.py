"""Pure deadline policy; callers supply monotonic seconds, never sleep here."""

import math


def validate_doorbell_timeout(timeout_s):
    if not math.isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError("doorbell_timeout_s must be finite and positive")
    return float(timeout_s)


def doorbell_wait_status(enabled, started_at, now, timeout_s, detected_at=None):
    """Return disabled/waiting/detected/timeout, ignoring events outside this wait."""
    if not enabled:
        return "disabled"
    timeout_s = validate_doorbell_timeout(timeout_s)
    if started_at is None:
        return "waiting"
    deadline = started_at + timeout_s
    if detected_at is not None and started_at <= detected_at <= now and detected_at < deadline:
        return "detected"
    return "timeout" if now >= deadline else "waiting"
