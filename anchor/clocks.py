"""Clock relations between boards. Line and band() are adopted from
esp-pingpong (box/taploc.py, Joannis Orlandos): a least-squares line with
median-based outlier rejection and an error that grows away from the data."""
import math


class Line:
    """Clock relation from instants known on both clocks: pairs (t_a, t_b), us.
    pick_us: how well one instant is timed; floor_us: error that more instants
    do not average away."""
    DRIFT_PPM = 20.0  # how fast two crystals may part when drift is not measured

    def __init__(self, pairs, pick_us, floor_us=0.0):
        self.floor = floor_us
        self.t0 = sum(p[0] for p in pairs) / len(pairs)
        rows = [((p[0] - self.t0) * 1e-6, p[1] - p[0]) for p in pairs]
        self._solve(rows, pick_us)
        for _ in range(4):
            if self.n < 8:
                break
            miss = sorted(abs(y - self.offset - self.drift * x) for x, y in rows)
            limit = 4 * (1.4826 * miss[len(miss) // 2] + 1.0)
            keep = [(x, y) for x, y in rows if abs(y - self.offset - self.drift * x) <= limit]
            if not 6 <= len(keep) < len(rows):
                break
            rows = keep
            self._solve(rows, pick_us)

    def _solve(self, rows, pick_us):
        self.n = len(rows)
        mx = sum(x for x, _ in rows) / self.n
        self.sxx = sum((x - mx) ** 2 for x, _ in rows)
        self.mx = mx
        my = sum(y for _, y in rows) / self.n
        self.measured = self.n >= 2 and self.sxx >= 8.0
        self.drift = sum((x - mx) * (y - my) for x, y in rows) / self.sxx if self.measured else 0.0
        self.offset = my - self.drift * mx
        self.s = pick_us
        if self.n > 2:
            miss = sum((y - self.offset - self.drift * x) ** 2 for x, y in rows)
            self.s = max(self.s, math.sqrt(miss / (self.n - 2)))

    def __call__(self, t):
        """t on clock a -> (t on clock b, 1-sigma us)."""
        x = (t - self.t0) * 1e-6
        if self.measured:
            var = self.s ** 2 * (1 / self.n + (x - self.mx) ** 2 / self.sxx)
        else:
            var = self.s ** 2 / self.n + (self.DRIFT_PPM * (x - self.mx)) ** 2
        return t + self.offset + self.drift * x, math.sqrt(var + self.floor ** 2)

    @property
    def ppm(self):
        return self.drift if self.measured else None


def band(pairs, share=0.4, floor_us=0.0):
    """Line through the densest band of the pairs' gaps: most one-way delays
    crowd together and stragglers trail off to one side."""
    line = Line(pairs, 1.0, floor_us)
    count = max(5, int(share * len(pairs)))
    if len(pairs) <= count:
        return line
    for _ in range(3):
        gaps = sorted((p[1] - line(p[0])[0], p) for p in pairs)
        start = min(range(len(gaps) - count + 1), key=lambda i: gaps[i + count - 1][0] - gaps[i][0])
        line = Line([p for _, p in gaps[start:start + count]], 1.0, floor_us)
    return line
