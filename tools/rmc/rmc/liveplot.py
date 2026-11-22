"""liveplotter：周期读取若干参数并实时画曲线（需要 matplotlib）。"""

import time
from collections import deque

from .errors import EpError


def run(device, names, rate=10.0, window=10.0, duration=None):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        raise EpError("liveplotter 需要 matplotlib：pip install robotmc[plot]")

    from .proxy import resolve

    names = [resolve(device, n) for n in names]
    if not device.descriptor:
        device.fetch_descriptor()

    points = max(16, int(window * rate))
    series = {name: deque(maxlen=points) for name in names}
    times = deque(maxlen=points)

    fig, ax = plt.subplots()
    lines = {}
    for name in names:
        line, = ax.plot([], [], label=name)
        lines[name] = line
    ax.set_xlabel("t (s)")
    ax.set_ylabel("value")
    ax.grid(True, alpha=0.3)
    if len(names) > 1:
        ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.canvas.draw()

    start = time.monotonic()
    try:
        while duration is None or time.monotonic() - start < duration:
            if not plt.fignum_exists(fig.number):
                break
            times.append(time.monotonic() - start)
            for name in names:
                try:
                    value = float(device.read(name))
                except EpError:
                    value = float("nan")
                series[name].append(value)
                lines[name].set_data(list(times), list(series[name]))
            ax.relim()
            ax.autoscale_view()
            fig.canvas.draw_idle()
            plt.pause(max(0.001, min(0.05, 1.0 / rate)))
    except KeyboardInterrupt:
        pass
    finally:
        plt.close(fig)
