"""Seleccion visual GPSR sin ROS; solo coincidencias confirmadas con posicion valida."""
import math

from .person_matcher import match_person


def valid_descriptor(descriptor):
    return (isinstance(descriptor, dict) and bool(descriptor)
            and set(descriptor) <= {'gesture', 'posture', 'clothing'}
            and all(isinstance(v, str) and v.strip() for v in descriptor.values()))


class PersonSearch:
    """Agrupa coincidencias cercanas y elige la mas cercana al robot.

    update_frame recibe observaciones de una sola imagen: una pista cuenta como
    maximo una vez por imagen. UNKNOWN y NO_MATCH nunca suman confirmaciones.
    """
    def __init__(self, descriptor, min_frames=3, match_dist_m=0.8):
        self.descriptor = descriptor
        self.min_frames = max(1, int(min_frames))
        self.match_dist_m = match_dist_m
        self.tracks = []

    def update_frame(self, candidates):
        if not valid_descriptor(self.descriptor):
            return
        used = set()
        for candidate in candidates:
            try:
                xy = [float(v) for v in candidate.get('xy', ())]
            except (TypeError, ValueError):
                continue
            if len(xy) != 2 or not all(math.isfinite(v) for v in xy):
                continue
            result = match_person(candidate, self.descriptor)
            if not result.is_match:
                continue
            nearest = min((i for i in range(len(self.tracks)) if i not in used),
                          key=lambda i: math.dist(self.tracks[i]['xy'], xy), default=None)
            if nearest is None or math.dist(self.tracks[nearest]['xy'], xy) > self.match_dist_m:
                nearest = len(self.tracks)
                self.tracks.append(dict(xy=xy, frames=0))
            track = self.tracks[nearest]
            n = track['frames']
            track['xy'] = [(a*n+b)/(n+1) for a, b in zip(track['xy'], xy)]
            track['frames'] += 1
            used.add(nearest)

    def build_response(self, request_id, robot_pos=(0., 0.)):
        confirmed = [t for t in self.tracks if t['frames'] >= self.min_frames]
        person = min(confirmed, key=lambda t: math.dist(t['xy'], robot_pos), default=None)
        reason = ('matched' if person else 'no_matching_person') if valid_descriptor(self.descriptor) else 'invalid_descriptor'
        return dict(id=request_id, ok=person is not None,
                    person=dict(xy=list(person['xy'])) if person else None,
                    reason=reason, error='' if person else reason)
