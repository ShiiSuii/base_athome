"""Personas vistas desde un lugar, con su posicion en el mapa. Sin ROS.

Restaurant: al girar en el lugar, cada persona aparece en varias vistas. Las
observaciones a menos de `match_dist_m` se juntan en una sola persona (mediana);
solo cuentan las vistas con al menos `min_hits` cuadros, para no tomar un falso
positivo de un solo cuadro. No exige mano levantada.
"""

import math
from statistics import median


class PeopleTally:
    def __init__(self, match_dist_m: float = 0.7):
        self.match_dist_m = float(match_dist_m)
        self.people: list[list[tuple[float, float]]] = []

    @staticmethod
    def _valid(xy) -> bool:
        try:
            return len(xy) == 2 and all(math.isfinite(float(v)) for v in xy)
        except (TypeError, ValueError):
            return False

    @staticmethod
    def _center(spots):
        return (median(s[0] for s in spots), median(s[1] for s in spots))

    def add(self, xy) -> None:
        if not self._valid(xy):
            return
        xy = (float(xy[0]), float(xy[1]))
        best, best_d = None, self.match_dist_m
        for spots in self.people:
            c = self._center(spots)
            d = math.hypot(c[0] - xy[0], c[1] - xy[1])
            if d < best_d:
                best, best_d = spots, d
        if best is None:
            self.people.append([xy])
        else:
            best.append(xy)

    def merge(self, other: "PeopleTally") -> None:
        for spots in other.people:
            for xy in spots:
                self.add(xy)

    def result(self, robot_xy, min_hits: int = 2) -> list[dict]:
        """Personas con al menos `min_hits` observaciones, de la mas cercana a la mas lejana."""
        rx, ry = float(robot_xy[0]), float(robot_xy[1])
        out = []
        for spots in self.people:
            if len(spots) < max(1, int(min_hits)):
                continue
            x, y = self._center(spots)
            out.append({"xy": [x, y], "range_m": math.hypot(x - rx, y - ry), "frames": len(spots)})
        return sorted(out, key=lambda p: p["range_m"])
