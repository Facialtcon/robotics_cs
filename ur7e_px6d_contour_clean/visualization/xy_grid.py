"""Synchronized physical tick intervals for equal-aspect Base XY plots."""
import math

from matplotlib.ticker import Locator, MultipleLocator


class _XYLocator(Locator):
    def __call__(self):
        axes = self.axis.axes
        # Use the same pixel spacing in X and Y; recompute on every redraw.
        raw = 86 * max(abs(b-a)/max(pixels, 1.) for (a, b), pixels in
                       ((axes.get_xlim(), axes.bbox.width),
                        (axes.get_ylim(), axes.bbox.height)))
        base = 10. ** math.floor(math.log10(max(raw, 1e-12)))
        step = next(v for v in (1, 2, 5, 10) if v*base >= raw-1e-12)*base
        return self.raise_if_exceeds(MultipleLocator(step).tick_values(*self.axis.get_view_interval()))


def square_xy_ticks(axes):
    """Keep independent locator instances, sharing the physical step calculation."""
    axes.xaxis.set_major_locator(_XYLocator())
    axes.yaxis.set_major_locator(_XYLocator())
