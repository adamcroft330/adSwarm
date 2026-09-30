"""Generate the hero artwork: streamlines of potential flow around an airfoil.

The airfoil is a Joukowski section. Flow is solved around an offset circle in
the zeta-plane (uniform stream + doublet + circulation, with the Kutta condition
fixing the circulation), streamlines are traced there with RK4 and then mapped
to the airfoil plane with z = zeta + c^2 / zeta. Output is a self-contained SVG
with a gentle CSS "pulse" travelling along a subset of the lines.

Pure standard library so the site builds anywhere Python 3 runs.
"""

import cmath
import math

# --- Geometry -------------------------------------------------------------
C = 1.0                      # Joukowski constant; chord is roughly 4C
MU = complex(-0.09, 0.11)    # circle centre offset -> thickness and camber
R = abs(C - MU)              # circle passes through the trailing edge (zeta = C)
ALPHA = math.radians(7.0)    # angle of attack
U = 1.0

# Kutta condition: stagnation point at the trailing edge.
_te = C - MU
_g = 2j * math.pi * _te * U * (cmath.exp(-1j * ALPHA) - R * R * cmath.exp(1j * ALPHA) / (_te * _te))
GAMMA = _g.real  # the imaginary part is zero to rounding


def dw_dzeta(zeta):
    """Complex velocity (u - iv) in the circle plane."""
    s = zeta - MU
    return (U * (cmath.exp(-1j * ALPHA) - R * R * cmath.exp(1j * ALPHA) / (s * s))
            + 1j * GAMMA / (2 * math.pi * s))


def to_z(zeta):
    return zeta + C * C / zeta


def _direction(zeta):
    v = dw_dzeta(zeta).conjugate()
    m = abs(v)
    return v / m if m > 1e-9 else 0j


def trace(zeta0, ds=0.02, max_steps=4000, x_exit=4.4):
    """Trace one streamline in the circle plane; return points in the z plane."""
    pts = [to_z(zeta0)]
    zeta = zeta0
    for _ in range(max_steps):
        k1 = _direction(zeta)
        k2 = _direction(zeta + 0.5 * ds * k1)
        k3 = _direction(zeta + 0.5 * ds * k2)
        k4 = _direction(zeta + ds * k3)
        step = (k1 + 2 * k2 + 2 * k3 + k4) / 6
        if abs(step) < 1e-9:
            break
        zeta = zeta + ds * step
        # keep numerical drift from sneaking inside the body
        s = zeta - MU
        if abs(s) < R * 1.0005:
            zeta = MU + s / abs(s) * R * 1.0005
        z = to_z(zeta)
        pts.append(z)
        if z.real > x_exit:
            break
    return pts


def airfoil(n=240):
    return [to_z(MU + R * cmath.exp(1j * 2 * math.pi * k / n)) for k in range(n)]


# --- Polyline simplification (Ramer-Douglas-Peucker) ------------------------
def _rdp(points, eps):
    if len(points) < 3:
        return points
    (x1, y1), (x2, y2) = points[0], points[-1]
    dx, dy = x2 - x1, y2 - y1
    norm = math.hypot(dx, dy) or 1e-12
    idx, dmax = 0, 0.0
    for i in range(1, len(points) - 1):
        x0, y0 = points[i]
        d = abs(dy * x0 - dx * y0 + x2 * y1 - y2 * x1) / norm
        if d > dmax:
            idx, dmax = i, d
    if dmax > eps:
        left = _rdp(points[: idx + 1], eps)
        right = _rdp(points[idx:], eps)
        return left[:-1] + right
    return [points[0], points[-1]]


def _path(points):
    head = "M{:.1f} {:.1f}".format(*points[0])
    return head + "".join("L{:.1f} {:.1f}".format(x, y) for x, y in points[1:])


# --- SVG ------------------------------------------------------------------
W, H = 1600, 900
SCALE = 165          # px per unit in the z plane
CX, CY = 1140, 455   # where the z-plane origin lands in the SVG


_LEVEL = cmath.exp(-1j * ALPHA)  # rotate so the freestream runs horizontally


def _px(z):
    z = z * _LEVEL
    return (CX + SCALE * z.real, CY - SCALE * z.imag)


def build_svg():
    x_start = -(CX / SCALE) - 0.6
    y_top = CY / SCALE + 0.8
    y_bot = -(H - CY) / SCALE - 0.8
    n_lines = 44
    x_exit = (W - CX) / SCALE + 0.4

    lines = []
    for i in range(n_lines):
        y = y_bot + (y_top - y_bot) * i / (n_lines - 1)
        pts = trace(complex(x_start, y), x_exit=x_exit)
        px = [_px(z) for z in pts]
        px = _rdp(px, 0.35)
        # lines passing close to the body read brighter
        dist = min(abs(z - complex(0.0, 0.25)) for z in pts)
        lines.append((_path(px), dist))

    foil = [_px(z) for z in airfoil()]
    foil_d = _path(foil) + "Z"

    out = [
        '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w} {h}" '
        'preserveAspectRatio="xMidYMid slice" aria-hidden="true">'.format(w=W, h=H),
        "<style>"
        ".s{fill:none;stroke:#7fdcf2;stroke-width:1.1;vector-effect:non-scaling-stroke}"
        ".p{fill:none;stroke:#b8f1ff;stroke-width:1.8;stroke-linecap:round;"
        "stroke-dasharray:36 964;stroke-dashoffset:1000;animation:f 9s linear infinite}"
        "@keyframes f{to{stroke-dashoffset:0}}"
        "@media (prefers-reduced-motion:reduce){.p{animation:none;opacity:0}}"
        "</style>",
        "<defs>"
        '<linearGradient id="fg" x1="0" y1="0" x2="1" y2="1">'
        '<stop offset="0" stop-color="#1c3f5e"/><stop offset="1" stop-color="#0d2336"/>'
        "</linearGradient>"
        '<radialGradient id="glow" cx="0.68" cy="0.5" r="0.45">'
        '<stop offset="0" stop-color="#1aa8cc" stop-opacity="0.22"/>'
        '<stop offset="1" stop-color="#1aa8cc" stop-opacity="0"/>'
        "</radialGradient>"
        "</defs>",
        '<rect width="{w}" height="{h}" fill="url(#glow)"/>'.format(w=W, h=H),
    ]

    for d, dist in lines:
        op = max(0.10, min(0.55, 0.62 - 0.16 * dist))
        out.append('<path class="s" d="{}" opacity="{:.2f}"/>'.format(d, op))

    # pulses on every third line, staggered so they never march in step
    for n, (d, dist) in enumerate(lines):
        if n % 3 != 1:
            continue
        delay = -((n * 1.37) % 9.0)
        dur = 7.5 + (n * 0.53) % 3.0
        out.append(
            '<path class="p" d="{}" pathLength="1000" '
            'style="animation-delay:{:.2f}s;animation-duration:{:.2f}s" opacity="{:.2f}"/>'.format(
                d, delay, dur, max(0.35, min(0.95, 1.0 - 0.2 * dist))
            )
        )

    out.append('<path d="{}" fill="url(#fg)" stroke="#3ccfef" stroke-width="1.5"/>'.format(foil_d))
    out.append("</svg>")
    return "\n".join(out)


if __name__ == "__main__":
    import sys

    print("Gamma = {:.4f} (imag residue {:.2e})".format(GAMMA, _g.imag), file=sys.stderr)
    print("TE velocity = {:.2e}".format(abs(dw_dzeta(C))), file=sys.stderr)
    sys.stdout.write(build_svg())
