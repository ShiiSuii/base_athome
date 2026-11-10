"""Personas vistas desde un lugar, con su posicion en el mapa. Sin ROS.

Restaurant: al girar en el lugar, cada persona aparece en varias vistas. Las
observaciones a menos de `match_dist_m` se juntan en una sola persona (mediana);
solo cuentan las vistas con al menos `min_hits` cuadros, para no tomar un falso
positivo de un solo cuadro. No exige mano levantada: cuenta en cuantos cuadros
la tenia y el angulo respecto del frente del robot (grados, + a la izquierda),
que no depende del mapa: el yaw de SLAM se corre al girar en el lugar.
"""

import math
from statistics import median


class PeopleTally:
    def __init__(self, match_dist_m: float = 0.7):
        self.match_dist_m = float(match_dist_m)
        self.people: list[list[tuple[float, float]]] = []
        self.raised: list[int] = []      # cuadros con mano levantada, por persona
        self.bearings: list[list[float]] = []  # grados respecto del frente del robot, por persona

    @staticmethod
    def _valid(xy) -> bool:
        try:
            return len(xy) == 2 and all(math.isfinite(float(v)) for v in xy)
        except (TypeError, ValueError):
            return False

    @staticmethod
    def _center(spots):
        return (median(s[0] for s in spots), median(s[1] for s in spots))

    def add(self, xy, *, raised: bool = False, bearing_deg=None) -> None:
        if not self._valid(xy):
            return
        xy = (float(xy[0]), float(xy[1]))
        best, best_d = None, self.match_dist_m
        for i, spots in enumerate(self.people):
            c = self._center(spots)
            d = math.hypot(c[0] - xy[0], c[1] - xy[1])
            if d < best_d:
                best, best_d = i, d
        if best is None:
            self.people.append([])
            self.raised.append(0)
            self.bearings.append([])
            best = len(self.people) - 1
        self.people[best].append(xy)
        self.raised[best] += int(bool(raised))
        if bearing_deg is not None and math.isfinite(bearing_deg):
            self.bearings[best].append(float(bearing_deg))

    def result(self, robot_xy, min_hits: int = 2) -> list[dict]:
        """Personas con al menos `min_hits` observaciones, de la mas cercana a la mas lejana."""
        rx, ry = float(robot_xy[0]), float(robot_xy[1])
        out = []
        for spots, raised, bearings in zip(self.people, self.raised, self.bearings):
            if len(spots) < max(1, int(min_hits)):
                continue
            x, y = self._center(spots)
            person = {"xy": [x, y], "range_m": math.hypot(x - rx, y - ry), "frames": len(spots),
                      "raised_frames": raised}
            if bearings:
                person["bearing_deg"] = median(bearings)
            out.append(person)
        return sorted(out, key=lambda p: p["range_m"])
