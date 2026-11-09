"""Guardas puras de frescura para docking, sin ROS ni reloj global."""

from typing import Optional


DOCK_SENSOR_MAX_AGE_S = 0.3


def sensor_freshness_error(
    now: float, scan_received_at: Optional[float], odom_received_at: Optional[float],
) -> Optional[str]:
    """Exige ambas llegadas en los últimos 0.3 s del mismo reloj monotónico.

    No usa el stamp ROS: el tiempo simulado puede detenerse. None significa
    que ese sensor aún no llegó; edades negativas o no finitas son inseguras.
    """
    for sensor, received_at in (("scan", scan_received_at), ("odom", odom_received_at)):
        if received_at is None:
            return f"no_{sensor}"
        if not 0.0 <= now - received_at <= DOCK_SENSOR_MAX_AGE_S:
            return f"stale_{sensor}"
    return None
