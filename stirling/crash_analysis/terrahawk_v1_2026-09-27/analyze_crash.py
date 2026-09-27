#!/usr/bin/env python3
"""Crash analysis: TerraHawk v1 QuadPlane, QuickTune hover crash (log 00000008.BIN).

Reproduces every number and figure quoted in README.md from the ArduPilot
DataFlash log:

    python -m pip install pymavlink numpy scipy matplotlib
    python analyze_crash.py /path/to/00000008.BIN            # prints summary, writes figures/

Parsing the 55 MB log takes well under a minute; the parsed arrays are cached in
the system temp dir so re-runs are quicker still.
"""
import argparse
import hashlib
import os
import pickle
import tempfile
from collections import defaultdict

import numpy as np
from scipy import signal

G = 9.80665
MSGS = ['PARM', 'MSG', 'QWIK', 'RATE', 'ATT', 'IMU', 'RCOU', 'RCIN', 'BAT',
        'QTUN', 'MOTB', 'PIQR', 'PIQP', 'PIQY', 'VIBE', 'MCU', 'ARSP', 'PSCD', 'XKF1', 'STAT']
# QUAD/X output mapping on this airframe: SERVO5=M1, SERVO4=M2, SERVO3=M3, SERVO6=M4
MOTORS = [('M1', 'C5', 'front-right, CCW'), ('M2', 'C4', 'rear-left, CCW'),
          ('M3', 'C3', 'front-left, CW'), ('M4', 'C6', 'rear-right, CW')]


# ----------------------------------------------------------------------------
# Log loading
# ----------------------------------------------------------------------------
def load(path):
    st = os.stat(path)
    key = hashlib.sha1(f'{os.path.abspath(path)}:{st.st_size}:{st.st_mtime}:{MSGS}'.encode()).hexdigest()[:12]
    cache = os.path.join(tempfile.gettempdir(), f'crashlog_{key}.pkl')
    if os.path.exists(cache):
        with open(cache, 'rb') as f:
            return pickle.load(f)
    from pymavlink import mavutil
    mlog = mavutil.mavlink_connection(path, dialect='ardupilotmega')
    cols = defaultdict(lambda: defaultdict(list))
    params, texts = {}, []
    while True:
        msg = mlog.recv_match(type=MSGS)
        if msg is None:
            break
        typ = msg.get_type()
        if typ == 'PARM':
            params[msg.Name] = msg.Value
            continue
        if typ == 'MSG':
            texts.append((msg.TimeUS / 1e6, msg.Message))
            continue
        for k, v in msg.to_dict().items():
            if k != 'mavpackettype':
                cols[typ][k].append(v)
    data = {t: {k: np.asarray(v) for k, v in c.items()} for t, c in cols.items()}
    out = (data, params, texts)
    with open(cache, 'wb') as f:
        pickle.dump(out, f)
    return out


class Log:
    def __init__(self, data, params, texts):
        self.D, self.P, self.texts = data, params, texts

    def t(self, msg):
        return self.D[msg]['TimeUS'] / 1e6

    def mask(self, msg, t0, t1, **eq):
        tt = self.t(msg)
        m = (tt >= t0) & (tt < t1)
        for k, v in eq.items():
            m &= self.D[msg][k] == v
        return m

    def at(self, msg, field, t):
        tt = self.t(msg)
        i = min(max(np.searchsorted(tt, t), 0), len(tt) - 1)
        return self.D[msg][field][i]

    def mean(self, msg, field, t0, t1, **eq):
        return float(self.D[msg][field][self.mask(msg, t0, t1, **eq)].mean())

    def text_time(self, needle, after=0.0):
        for t, s in self.texts:
            if t >= after and needle in s:
                return t
        return None

    def thrust(self, pwm):
        """Commanded thrust fraction (0..1) from motor PWM using the log's own motor params."""
        P = self.P
        a = (np.asarray(pwm, float) - P['Q_M_PWM_MIN']) / (P['Q_M_PWM_MAX'] - P['Q_M_PWM_MIN'])
        a = np.clip((a - P['Q_M_SPIN_MIN']) / (P['Q_M_SPIN_MAX'] - P['Q_M_SPIN_MIN']), 0, 1)
        e = P['Q_M_THST_EXPO']
        return e * a * a + (1 - e) * a

    def imu0(self):
        m = self.D['IMU']['I'] == 0
        return self.t('IMU')[m], {k: self.D['IMU'][k][m] for k in ('AccX', 'AccY', 'AccZ', 'GyrX', 'GyrY', 'GyrZ')}

    def lift_over_weight(self):
        """Lift / weight from the EKF vertical acceleration: 1 - (dVD/dt)/g.

        Uses core 0 VD (m/s down) at 50 Hz, differentiated with a 0.3 s Savitzky-Golay window.
        Vertical speeds stay below ~1 m/s until the failure, so drag is negligible here.
        """
        m = self.D['XKF1']['C'] == 0
        t, vd = self.t('XKF1')[m], self.D['XKF1']['VD'][m]
        dt = np.median(np.diff(t))
        acc_down = signal.savgol_filter(vd, 15, 2, deriv=1, delta=dt)
        return t, 1.0 - acc_down / G

    def specific_force_g(self):
        """|accelerometer| / g from IMU0: ~1 while lift carries the weight, ~0 in free fall."""
        t, a = self.imu0()
        return t, np.sqrt(a['AccX'] ** 2 + a['AccY'] ** 2 + a['AccZ'] ** 2) / G


def smooth(x, n):
    if n <= 1:
        return x
    return np.convolve(x, np.ones(n) / n, mode='same')


# ----------------------------------------------------------------------------
# Analysis
# ----------------------------------------------------------------------------
def analyse(L):
    R = {}
    # Arm periods from the text log
    arms = [t for t, s in L.texts if s == 'Throttle armed']
    disarms = [t for t, s in L.texts if s == 'Throttle disarmed']
    R['abort'] = L.text_time('ABORTING')
    R['arm2'] = max(a for a in arms if a < R['abort'])   # arming of the crash flight
    R['disarm_final'] = max(disarms)
    R['thrust_loss_msg'] = L.text_time('Thrust Loss')

    # QuickTune stages, final attempt of flight 2 (the one that ran to the abort)
    q, tq = L.D['QWIK'], L.t('QWIK')
    last_start = max(t for t, s in L.texts if s == 'Quicktune: Starting tune' and t < R['abort'])
    stages = []
    for name in ('Roll D', 'Roll P', 'Pitch D', 'Pitch P'):
        m = (q['Param'] == name) & (tq >= last_start - 0.1) & (tq < R['abort'])
        tt, gg, ss = tq[m], q['Gain'][m], q['SRate'][m]
        k = int(np.argmax(gg))
        start = float(gg[0])
        final = max(0.4 * gg[k], start * (1 - L.P['QWIK_REDUCE_MAX'] / 100.0))
        stages.append(dict(name=name, t0=float(tt[0]), t_trip=float(tt[k]), start=start,
                           trip=float(gg[k]), sr=float(ss[k]), final=float(final), t=tt, g=gg))
    R['stages'] = stages

    # Built-in thrust imbalance in steady hover before any tuning (windows picked for this log:
    # airborne and settled, before the first QuickTune start of each flight)
    R['hover'] = {}
    for label, (a, b) in (('flight 1 pre-tune', (66, 73.5)), ('flight 2 pre-tune', (196, 214))):
        mo = L.mask('RCOU', a, b)
        thr = {n: float(L.thrust(L.D['RCOU'][c][mo]).mean()) for n, c, _ in MOTORS}
        pwm = {n: float(L.D['RCOU'][c][mo].mean()) for n, c, _ in MOTORS}
        mr = L.mask('RATE', a, b)
        R['hover'][label] = dict(thr=thr, pwm=pwm, ROut=float(L.D['RATE']['ROut'][mr].mean()),
                                 POut=float(L.D['RATE']['POut'][mr].mean()),
                                 YOut=float(L.D['RATE']['YOut'][mr].mean()))

    # Failure detection
    t, a = L.imu0()
    b, a_ = signal.butter(4, 15, btype='high', fs=300)
    hp = np.sqrt(sum(signal.filtfilt(b, a_, a[k]) ** 2 for k in ('AccX', 'AccY', 'AccZ')))
    amag = np.sqrt(a['AccX'] ** 2 + a['AccY'] ** 2 + a['AccZ'] ** 2)
    fl = (t > R['abort'] - 2) & (t < R['abort'] + 3)
    R['impact'] = float(t[fl][np.argmax(amag[fl])])
    m = (t > R['abort'] - 1.0) & (t < R['abort'])
    R['shock1'] = float(t[m][np.argmax(hp[m] > 4.0)])
    m = (t > R['shock1'] + 0.5) & (t < R['impact'] - 0.1)
    R['shock2'] = float(t[m][np.argmax(hp[m])])
    mi = (t >= R['impact'] - 0.025) & (t < R['impact'] + 0.025)
    R['impact_g_50ms'] = float(amag[mi].mean() / G)
    R['clip_total'] = int(L.D['VIBE']['Clip'][L.D['VIBE']['IMU'] == 0].max())

    # Current collapse: first 10 Hz BAT sample after shock that is < 3 A
    tb, cur = L.t('BAT'), L.D['BAT']['Curr']
    m = tb > R['shock1']
    R['power_lost_sample'] = float(tb[m][np.argmax(cur[m] < 3.0)])
    R['cur_before'] = float(cur[(tb > R['shock1'] - 0.2) & (tb < R['shock1'])].max())
    burst = (tb > R['power_lost_sample']) & (tb < R['impact'])
    R['burst_t'] = float(tb[burst][np.argmax(cur[burst])])
    R['burst_A'] = float(cur[burst].max())
    grd = (tb > R['impact'] + 5) & (tb < R['disarm_final'])
    R['ground_cur_max'] = float(cur[grd].max())
    R['ground_cur_mean'] = float(cur[grd].mean())
    R['disarmed_baseline'] = L.mean('BAT', 'Curr', R['disarm_final'] + 2, R['disarm_final'] + 14)

    # Height above take-off, fall
    alt0 = L.mean('QTUN', 'Alt', R['arm2'], R['arm2'] + 1.5)   # on the ground, just armed
    R['alt0'] = alt0
    R['h_shock'] = float(L.at('QTUN', 'Alt', R['shock1']) - alt0)
    tq_, crt = L.t('QTUN'), L.D['QTUN']['CRt']
    R['impact_speed'] = float(-crt[(tq_ > R['shock1']) & (tq_ < R['impact'] + 0.1)].min())

    # Thrust efficiency: achieved lift/weight divided by commanded collective / hover
    # Lift per window from a straight-line fit of EKF VD inside the window only, so that the
    # last pre-failure window is not contaminated by free-fall samples after the shock.
    mk = L.D['XKF1']['C'] == 0
    tk, vd = L.t('XKF1')[mk], L.D['XKF1']['VD'][mk]
    rows = []  # (t0, t1, commanded collective / hover, lift / weight, efficiency, current)
    wins = [(a0, a0 + 4) for a0 in range(196, 296, 4)] + [(a0, a0 + 1) for a0 in range(296, 308)]
    wins += [(308.0, 308.5), (308.5, 309.0), (309.0, R['shock1'])]   # 0.5 s: shorter windows are too noisy
    for a0, a1 in wins:
        mm = (tk >= a0) & (tk < a1)
        cmd = L.mean('QTUN', 'ThO', a0, a1) / L.mean('QTUN', 'ThH', a0, a1)
        lift = float(1.0 - np.polyfit(tk[mm], vd[mm], 1)[0] / G)
        rows.append((a0, a1, cmd, lift, lift / cmd, L.mean('BAT', 'Curr', a0, a1)))
    R['eff'] = rows

    # High-frequency (>15 Hz) vibration precursor
    R['hf'] = [(w, float(np.sqrt((hp[(t >= w) & (t < w + 0.05)] ** 2).mean())))
               for w in np.arange(R['shock1'] - 0.5, R['shock1'] + 0.05, 0.05)]
    R['hf_baseline'] = float(np.sqrt((hp[(t > stages[0]['t0']) & (t < R['shock1'] - 0.5)] ** 2).mean()))

    # Rate-tracking error (1 s RMS) for the overview
    r, tr = L.D['RATE'], L.t('RATE')
    err = []
    for w in np.arange(R['arm2'] + 5, R['shock1'] - 1.0, 1.0):   # every window ends before the shock
        mm = (tr >= w) & (tr < w + 1)
        err.append((w + 0.5, float(np.sqrt(((r['R'][mm] - r['RDes'][mm]) ** 2).mean())),
                    float(np.sqrt(((r['P'][mm] - r['PDes'][mm]) ** 2).mean()))))
    R['rate_err'] = np.array(err)
    return R


def summary(L, R):
    P = L.P
    fw = next((s for _, s in L.texts if s.startswith('ArduPlane')), '?')
    board = next((s for _, s in L.texts if 'LUCID' in s or 'H7' in s), '?')
    frame = next((s.split(': ')[1] for _, s in L.texts if s.startswith('QuadPlane Frame')), '?')
    rcout = next((s.split(': ')[1] for _, s in L.texts if s.startswith('RCOut')), '?')
    print(f'Firmware: {fw}   Board: {board.split()[0]}   Frame: {frame}   Outputs: {rcout}')
    print('\nSetup parameters of interest')
    for k in ('INS_HNTCH_ENABLE', 'INS_HNTC2_ENABLE', 'SERVO_BLH_BDMASK', 'INS_GYRO_FILTER',
              'Q_M_BAT_VOLT_MAX', 'Q_M_BAT_VOLT_MIN', 'Q_M_THST_EXPO', 'Q_M_SPIN_MAX',
              'Q_A_RAT_RLL_P', 'Q_A_RAT_RLL_I', 'Q_A_RAT_RLL_D', 'Q_A_RAT_PIT_P', 'Q_A_RAT_PIT_I',
              'Q_A_RAT_PIT_D', 'Q_A_RAT_YAW_P', 'Q_A_RAT_YAW_I', 'QWIK_OSC_SMAX', 'QWIK_DOUBLE_TIME',
              'QWIK_GAIN_MARGIN', 'QWIK_ANGLE_MAX', 'QWIK_AUTO_FILTER', 'QWIK_RP_PI_RATIO'):
        print(f'  {k:18s} {P[k]:g}')

    print('\nQuickTune, flight 2 final attempt (gain x = relative to start)')
    print(f'  {"stage":8s} {"start":>7s} {"trip":>8s} {"x":>5s} {"trip t":>8s} {"final":>8s} {"x":>5s}')
    for s in R['stages']:
        print(f'  {s["name"]:8s} {s["start"]:7.4f} {s["trip"]:8.4f} {s["trip"] / s["start"]:5.1f} '
              f'{s["t_trip"]:8.2f} {s["final"]:8.4f} {s["final"] / s["start"]:5.2f}')

    print('\nBuilt-in thrust imbalance in steady hover (commanded thrust share)')
    for label, h in R['hover'].items():
        mean = np.mean(list(h['thr'].values()))
        shares = '  '.join(f'{n} {100 * v / mean:4.0f}% ({h["pwm"][n]:.0f}us)' for n, v in h['thr'].items())
        print(f'  {label}: {shares}  | M4/M1 {h["thr"]["M4"] / h["thr"]["M1"]:.2f} | '
              f'trims ROut {h["ROut"]:+.3f} POut {h["POut"]:+.3f} YOut {h["YOut"]:+.3f}')
        # QUAD/X mixer factors (normalised to 0.5): the trims should reproduce each motor's thrust
        factors = {'M1': (-.5, .5, .5), 'M2': (.5, -.5, .5), 'M3': (.5, .5, -.5), 'M4': (-.5, -.5, -.5)}
        parts = []
        for n, (fr, fp, fy) in factors.items():
            pred = mean + fr * h['ROut'] + fp * h['POut'] + fy * h['YOut']
            parts.append(f'{n} pitch {fp * h["POut"]:+.3f} yaw {fy * h["YOut"]:+.3f} -> {pred:.3f} (obs {h["thr"][n]:.3f})')
        print('    trims -> thrust: ' + '; '.join(parts))

    print('\nThrust efficiency = (lift/weight) / (commanded collective / hover)')
    for a0, a1, cmd, lift, eff, cur in R['eff']:
        if a0 >= 292 or a0 % 20 == 0:
            print(f'  {a0:7.2f}-{a1:7.2f}s  cmd {cmd:4.2f}x hover  lift {lift:4.2f} g  eff {eff:4.2f}  {cur:5.1f} A')

    print(f'\n>15 Hz vibration baseline (214-309 s): {R["hf_baseline"]:.2f} m/s^2 RMS; last 0.5 s before shock:')
    print('  ' + '  '.join(f'{w:.2f}:{v:.2f}' for w, v in R['hf']))

    print('\nFailure sequence')
    print(f'  structural shock onset        {R["shock1"]:8.3f} s  (height {R["h_shock"]:.1f} m above take-off)')
    print(f'  battery current {R["cur_before"]:.1f} A -> <3 A by 10 Hz sample {R["power_lost_sample"]:.3f} s')
    print(f'  QuickTune abort (att. error)  {R["abort"]:8.3f} s')
    print(f'  brief power return            {R["burst_t"]:8.3f} s  ({R["burst_A"]:.0f} A for ~0.1-0.2 s)')
    print(f'  second shock (in air)         {R["shock2"]:8.3f} s')
    print(f'  impact                        {R["impact"]:8.3f} s  ({R["impact_g_50ms"]:.1f} g mean over 50 ms, '
          f'descent {R["impact_speed"]:.1f} m/s; {R["clip_total"]} accel clip events)')
    print(f'  "Potential VTOL Thrust Loss"  {R["thrust_loss_msg"]:8.3f} s  (after impact)')
    print(f'  on ground armed until         {R["disarm_final"]:8.3f} s  (current max {R["ground_cur_max"]:.2f} A, '
          f'mean {R["ground_cur_mean"]:.2f} A; disarmed baseline {R["disarmed_baseline"]:.2f} A)')


def precursors(L, R):
    """How early did anything in the log show the mount failing? Onsets relative to the break."""
    brk = R['shock1']
    ref = (296.0, 307.5)  # Pitch P ramp before the final second: the same flight regime, rocking included
    print('\nPrecursors (lead time before the break at %.2f s)' % brk)

    # ArduPilot's own vibration metric (raw high-rate accel, >5 Hz) and the >15 Hz band of the logged IMU data
    def sustained_onset(tt, x, thr):
        above = x > thr
        if not above[-1]:
            return None
        i = len(above) - 1
        while i > 0 and above[i - 1]:
            i -= 1
        return tt[i]

    v, tv = L.D['VIBE'], L.t('VIBE')
    for imu in (0, 1):
        out = []
        for ax in ('VibeX', 'VibeY', 'VibeZ'):
            mb = (tv >= ref[0]) & (tv < ref[1]) & (v['IMU'] == imu)
            thr = v[ax][mb].mean() + 4 * v[ax][mb].std()
            m = (tv >= ref[1]) & (tv < brk) & (v['IMU'] == imu)
            on = sustained_onset(tv[m], v[ax][m], thr)
            out.append(f'{ax[-1]} {brk - on:.2f} s' if on is not None else f'{ax[-1]} none')
        print(f'  VIBE above baseline+4sd, IMU{imu}: ' + ', '.join(out))
    b, a = signal.butter(4, 15, btype='high', fs=300)
    im, ti = L.D['IMU'], L.t('IMU')
    for imu in (0, 1):
        m0 = im['I'] == imu
        t = ti[m0]
        edges = np.arange(ref[0], brk + 1e-9, 0.05)
        idx = np.digitize(t, edges)
        out = []
        for ax in ('AccX', 'AccY', 'AccZ', 'GyrX', 'GyrY', 'GyrZ'):
            x = signal.filtfilt(b, a, im[ax][m0])
            rms = np.array([np.sqrt((x[idx == k] ** 2).mean()) for k in range(1, len(edges))])
            cen = edges[:-1]
            thr = np.percentile(rms[cen < ref[1]], 99.9)
            late = (cen >= ref[1]) & (cen < brk - 0.05)
            on = sustained_onset(cen[late], rms[late], thr)
            out.append(f'{ax} {brk - on:.2f} s' if on is not None else f'{ax} none')
        print(f'  >15 Hz band above baseline p99.9, IMU{imu}: ' + ', '.join(out))

    # Thrust efficiency in running 0.5 s windows, mean of both EKF cores
    x, tx, q, tq = L.D['XKF1'], L.t('XKF1'), L.D['QTUN'], L.t('QTUN')

    def eff(a0, a1):
        mq = (tq >= a0) & (tq < a1)
        cmd = q['ThO'][mq].mean() / q['ThH'][mq].mean()
        lift = np.mean([1 - np.polyfit(tx[(tx >= a0) & (tx < a1) & (x['C'] == c)],
                                       x['VD'][(tx >= a0) & (tx < a1) & (x['C'] == c)], 1)[0] / G for c in (0, 1)])
        return lift / cmd
    steady = [eff(a0, a0 + 0.5) for a0 in np.arange(R['stages'][0]['t0'], R['stages'][-1]['t0'] - 0.5, 0.5)]
    ramp = [(a0, eff(a0, a0 + 0.5)) for a0 in np.arange(ref[0], ref[1], 0.5)]
    lo_t, lo = min(ramp, key=lambda r: r[1])
    print(f'  thrust efficiency: steady tuning min {min(steady):.2f}; Pitch P ramp min {lo:.2f} at {lo_t:.1f} s (rocking)')
    for a0 in np.arange(brk - 1.5, brk - 0.49, 0.25):
        e = eff(a0, a0 + 0.5)
        flag = '  <- below anything earlier' if e < lo else ''
        print(f'    window ending {brk - (a0 + 0.5):.2f} s before break: {e:.2f}{flag}')

    # Yaw opposing its demand: how often did that happen earlier without a failure?
    r, tr = L.D['RATE'], L.t('RATE')
    m = (tr >= R['stages'][0]['t0']) & (tr < ref[1])
    opp = (np.sign(r['Y'][m]) != np.sign(r['YDes'][m])) & (np.abs(r['Y'][m]) > 10) & (np.abs(r['YDes'][m]) > 10)
    t, runs, start = tr[m], [], None
    for k, o in enumerate(opp):
        if o and start is None:
            start = k
        elif not o and start is not None:
            if t[k - 1] - t[start] > 0.3:
                runs.append(t[start])
            start = None
    mf = (tr >= brk - 1.0) & (tr < brk)
    print(f'  yaw rate >10 deg/s against a >10 deg/s demand for >0.3 s: {len(runs)} earlier episodes '
          f'({", ".join(f"{x:.1f}" for x in runs)} s); max yaw rate earlier {np.abs(r["Y"][m]).max():.1f} deg/s '
          f'vs {np.abs(r["Y"][mf]).max():.1f} in the final second')

    # Structural / roll-loop mode frequency: a softening mount would pull it down
    m0 = im['I'] == 0
    freqs = []
    for a0, a1 in ((R['arm2'] + 5, R['stages'][0]['t0']), (R['stages'][0]['t0'], ref[0]), ref):
        mm = m0 & (ti >= a0) & (ti < a1)
        g = np.degrees(im['GyrX'][mm]) - np.degrees(im['GyrX'][mm]).mean()
        f, pxx = signal.welch(g, fs=300, nperseg=1024, nfft=8192)
        s_ = (f >= 15) & (f <= 40)
        freqs.append(f'{f[s_][np.argmax(pxx[s_])]:.1f} Hz ({a0:.0f}-{a1:.0f} s)')
    print('  roll 15-40 Hz mode: ' + ', '.join(freqs))
    o, to = L.D['RCOU'], L.t('RCOU')
    mo = (to > brk - 1.0) & (to < brk)
    print(f'  M4 highest command before the break: {o["C6"][mo].max()} us')


def causality(L, R):
    """Did the tune's oscillation break the mount, or did a failing mount cause the oscillation?

    Everything here uses causal (past-data-only) filters and trailing windows, so no filter can
    move a later effect earlier in time. Filter delay only makes the oscillation look later.
    """
    brk, fs = R['shock1'], 300
    r, tr = L.D['RATE'], L.t('RATE')
    t0 = R['stages'][-1]['t0'] - 2.0
    m = (tr >= t0 - 5) & (tr < brk)
    t = tr[m]
    b48, a48 = signal.butter(3, [4, 8], btype='band', fs=fs)
    b12, a12 = signal.butter(3, [1, 2.5], btype='band', fs=fs)
    pitch = signal.lfilter(b48, a48, r['P'][m] - r['PDes'][m])
    roll = signal.lfilter(b12, a12, r['R'][m] - r['RDes'][m])
    im, ti = L.D['IMU'], L.t('IMU')
    mi = (im['I'] == 0) & (ti >= t0 - 5) & (ti < brk)
    tv = ti[mi]
    bh, ah = signal.butter(4, 15, btype='high', fs=fs)
    vib = np.sqrt(sum(signal.lfilter(bh, ah, im[k][mi]) ** 2 for k in ('AccX', 'AccY', 'AccZ')))

    def trailing_rms(tt, x, ends, width):
        return np.array([np.sqrt((x[(tt > e - width) & (tt <= e)] ** 2).mean()) for e in ends])
    ends = np.arange(t0, brk + 1e-9, 0.05)
    C = dict(ends=ends, pitch=trailing_rms(t, pitch, ends, 0.4), roll=trailing_rms(t, roll, ends, 0.4),
             vib=trailing_rms(tv, vib, ends, 0.1))
    base = (ends >= R['stages'][-1]['t0']) & (ends < 307.5)
    C['vib_thr'] = float(np.percentile(C['vib'][base], 99.9))

    # thrust efficiency over trailing 0.5 s windows (mean of both EKF cores)
    x, tx, q, tq = L.D['XKF1'], L.t('XKF1'), L.D['QTUN'], L.t('QTUN')
    effs = []
    for e in ends:
        mq = (tq > e - 0.5) & (tq <= e)
        cmd = q['ThO'][mq].mean() / q['ThH'][mq].mean()
        lift = np.mean([1 - np.polyfit(tx[(tx > e - 0.5) & (tx <= e) & (x['C'] == c)],
                                       x['VD'][(tx > e - 0.5) & (tx <= e) & (x['C'] == c)], 1)[0] / G for c in (0, 1)])
        effs.append(lift / cmd)
    C['eff'] = np.array(effs)
    C['eff_prev_min'] = float(C['eff'][base].min())

    print('\nWhich came first? (causal filters, trailing windows)')
    q_, tq_ = L.D['QWIK'], L.t('QWIK')
    for a0, a1 in ((296.5, 299), (299, 301.5), (301.5, 304), (304, 306.5), (306.5, 308.5), (308.5, 309.0),
                   (309.0, 309.25), (309.25, brk)):
        k = (ends > a0) & (ends <= a1)
        mp = (q_['Param'] == 'Pitch P') & (tq_ >= a0) & (tq_ < a1)
        gain = f'{q_["Gain"][mp].min():.2f}-{q_["Gain"][mp].max():.2f}' if mp.any() else 'cut  '
        mo = (L.t('RCOU') >= a0) & (L.t('RCOU') < a1)
        print(f'  {a0:6.2f}-{a1:6.2f}  Pitch P {gain:>9}  pitch 4-8 Hz {C["pitch"][k].max():5.2f}  '
              f'roll 1-2.5 Hz {C["roll"][k].max():5.2f}  M4 max {L.D["RCOU"]["C6"][mo].max():4d} us  '
              f'vib>15Hz {C["vib"][k].max():4.2f} (normal <= {C["vib_thr"]:.2f})  eff min {C["eff"][k].min():.2f}')
    ipk = int(np.argmax(np.where(ends > R['stages'][-1]['t_trip'] - 0.5, C['pitch'], 0)))
    von = next(e for e, v in zip(ends[::-1], C['vib'][::-1]) if v <= C['vib_thr']) + 0.05
    print(f'  pitch oscillation peak: 0.4 s window centred on {ends[ipk] - 0.2:.2f} s ({C["pitch"][ipk]:.1f} deg/s), '
          f'{C["pitch"][-1]:.1f} at the break')
    print(f'  vibration leaves its normal range: 0.1 s window centred on {von - 0.05:.2f} s '
          f'({brk - (von - 0.05):.2f} s before the break)')
    # Is the final pitch oscillation the same mode that grew through the ramp? (per gain range)
    print('  pitch 4-8 Hz mode by Pitch P range (zero-phase within each window):')
    for a0, a1 in ((296.5, 299), (299, 301.5), (301.5, 304), (304, 306.5), (306.5, 308.5),
                   (308.5, R['stages'][-1]['t_trip'])):
        mm = (tr >= a0) & (tr < a1)
        e = r['P'][mm] - r['PDes'][mm]
        e = e - e.mean()
        f, pxx = signal.periodogram(e * np.hanning(len(e)), fs=fs, nfft=8192, scaling='spectrum')
        band = (f >= 4) & (f <= 8)
        amp = np.sqrt((signal.filtfilt(b48, a48, e) ** 2)[30:-30].mean())
        mp = (q_['Param'] == 'Pitch P') & (tq_ >= a0) & (tq_ < a1)
        print(f'    P {q_["Gain"][mp].min():.2f}-{q_["Gain"][mp].max():.2f}: peak {f[band][np.argmax(pxx[band])]:.1f} Hz, '
              f'{amp:.2f} deg/s RMS')
    R['causal'] = C


def weathervane(L, R):
    """Could weathervaning explain the yaw trim? Wind torque scales with wind^2 and heading; tilt scales with thrust."""
    r, tr, q, tq = L.D['RATE'], L.t('RATE'), L.D['QTUN'], L.t('QTUN')
    s, ts = L.D['ARSP'], L.t('ARSP')
    m0 = s['I'] == 0
    tsa, V = ts[m0], s['Airspeed'][m0]
    att, ta = L.D['ATT'], L.t('ATT')
    st, tst = L.D['STAT'], L.t('STAT')
    print('\nWeathervaning check (yaw output < 0 = counter-clockwise correction)')
    arms = [t for t, x in L.texts if x == 'Throttle armed']
    liftoffs = {}
    for arm in arms:
        k = (tst > arm) & (tst < arm + 15) & (st['isFlying'] == 1)
        if k.any():
            liftoffs.setdefault(round(float(tst[k][0]), 1), arm)   # one entry per lift-off
    for tl, arm in sorted(liftoffs.items()):
        arm = max(a for a in arms if a < tl)                      # the arming that led to this lift-off
        h0 = L.at('ATT', 'Yaw', arm + 0.5)
        k = (ta > tl) & (ta < tl + 4)
        swing = ((att['Yaw'][k] - h0 + 180) % 360 - 180).max()
        mr = (tr > tl + 2) & (tr < tl + 4)
        print(f'  lift-off {tl:6.1f} s at heading {h0:5.1f} deg: nose swung up to {swing:4.1f} deg clockwise in 4 s, '
              f'yaw output {r["YOut"][mr].mean():+.3f} within 2-4 s')
    alt0 = R['alt0']
    bands = []
    for lo, hi in ((1, 3), (9, 12)):
        mq = (tq > 205) & (tq < R['stages'][-1]['t0']) & (q['Alt'] - alt0 >= lo) & (q['Alt'] - alt0 < hi)
        bands.append(f'{lo}-{hi} m {np.mean(np.interp(tq[mq], tr, r["YOut"])):+.3f}')
    print('  yaw output by height (flight 2): ' + ', '.join(bands))
    for lab, (a0, a1) in (('flight 1', (75, 104)), ('flight 2', (215, R['stages'][-1]['t0']))):
        k = (ta >= a0) & (ta < a1)
        hd = np.degrees(np.angle(np.mean(np.exp(1j * np.radians(att['Yaw'][k]))))) % 360
        rows = []
        for t0 in np.arange(a0, a1, 1.0):
            mr = (tr >= t0) & (tr < t0 + 1); mq = (tq >= t0) & (tq < t0 + 1); ms = (tsa >= t0) & (tsa < t0 + 1)
            rows.append((r['YOut'][mr].mean(), np.mean(V[ms] ** 2), q['ThO'][mq].mean() / q['ThH'][mq].mean()))
        y, v2, thr = np.array(rows).T
        coef = np.linalg.lstsq(np.column_stack([np.ones_like(y), v2, thr]), y, rcond=None)[0]
        floor = np.mean(V[(tsa > R['impact'] + 10) & (tsa < R['disarm_final'])] ** 2)   # on the ground, no thrust
        wind = coef[1] * (v2.mean() - floor)
        print(f'  {lab}: heading {hd:5.1f} deg, mean yaw output {y.mean():+.3f}; gusts (pitot, above its '
              f'{np.sqrt(floor):.1f} m/s ground reading) explain {wind:+.3f} = {100 * wind / y.mean():.0f}% of it')


# ----------------------------------------------------------------------------
# Figures
# ----------------------------------------------------------------------------
SURF, INK, INK2, MUTED, GRID, AXIS = '#fcfcfb', '#0b0b0b', '#52514e', '#898781', '#e1e0d9', '#c3c2b7'
BLUE, VIOLET, AQUA, ORANGE = '#2a78d6', '#4a3aa7', '#1baf7a', '#eb6834'   # roll/X, pitch/Y, yaw/Z, M4
MGRAY = {'M1': '#c3c2b7', 'M2': '#9a9992', 'M3': '#6b6a65', 'M4': ORANGE}


def style():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        'figure.facecolor': SURF, 'axes.facecolor': SURF, 'savefig.facecolor': SURF,
        'axes.edgecolor': AXIS, 'axes.labelcolor': INK2, 'axes.titlecolor': INK, 'axes.titlesize': 10,
        'axes.titleweight': 'bold', 'axes.titlelocation': 'left', 'axes.labelsize': 8.5,
        'xtick.color': MUTED, 'ytick.color': MUTED, 'xtick.labelsize': 8, 'ytick.labelsize': 8,
        'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.7, 'grid.linestyle': '-',
        'axes.spines.top': False, 'axes.spines.right': False, 'axes.linewidth': 0.8,
        'lines.linewidth': 1.3, 'lines.solid_capstyle': 'round', 'lines.solid_joinstyle': 'round',
        'legend.frameon': False, 'legend.fontsize': 8, 'legend.labelcolor': INK2,
        'font.family': 'DejaVu Sans', 'text.color': INK,
    })
    return plt


def mark(axs, events, label_ax=0):
    for i, ax in enumerate(axs):
        for x, lab in events:
            ax.axvline(x, color=INK2, lw=0.8, alpha=0.7, zorder=1)
            if i == label_ax and lab:
                ax.text(x, 0.97, lab + ' ', transform=ax.get_xaxis_transform(), fontsize=7.5, color=INK2,
                        rotation=90, ha='right', va='top', zorder=5,
                        bbox=dict(boxstyle='square,pad=0.1', fc=SURF, ec='none', alpha=0.85))


def motor_lines(ax, L, t0, t1, mean_window=None, label_end=True):
    to = L.t('RCOU')
    for n, c, desc in MOTORS:
        m = (to >= t0) & (to < t1)
        x, y = to[m], L.D['RCOU'][c][m].astype(float)
        if mean_window:
            edges = np.arange(t0, t1 + 1e-9, mean_window)
            idx = np.digitize(x, edges)
            x = np.array([x[idx == k].mean() for k in np.unique(idx)])
            y = np.array([y[idx == k].mean() for k in np.unique(idx)])
        ax.plot(x, y, color=MGRAY[n], lw=2.0 if n == 'M4' else 1.1, zorder=3 if n == 'M4' else 2,
                label=f'{n} ({desc})')
        if label_end:
            ax.text(x[-1] + (t1 - t0) * 0.006, y[-1], n, color=INK2, fontsize=7.5, va='center')


def fig_overview(L, R, out):
    plt = style()
    t0, t1 = R['arm2'] - 2, R['impact'] + 1.0
    fig, axs = plt.subplots(5, 1, figsize=(12, 12.5), sharex=True,
                            gridspec_kw=dict(height_ratios=[1, 1, 1, 1.25, 0.8]))
    fig.suptitle('Flight 2: QuickTune progression, attitude tracking and motor load', x=0.01, ha='left',
                 fontsize=12.5, fontweight='bold')
    # a) height with tuning stages
    ax = axs[0]
    tq = L.t('QTUN'); m = (tq >= t0) & (tq < t1)
    ax.plot(tq[m], L.D['QTUN']['Alt'][m] - R['alt0'], color=INK)
    ax.set_ylabel('Height above\ntake-off (m)')
    ax.set_title('a  Height, with QuickTune stages shaded', fontsize=9.5)
    for s in R['stages']:
        end = s['t_trip']
        for a in axs:
            a.axvspan(s['t0'], end, color='#efeee9', zorder=0, lw=0)
        ax.text((s['t0'] + end) / 2, 0.93, s['name'], transform=ax.get_xaxis_transform(), ha='center',
                va='top', fontsize=8, color=INK2)
    ax.set_ylim(-1, 13)
    # b) gain multiplier
    ax = axs[1]
    for s in R['stages']:
        col = BLUE if s['name'].startswith('Roll') else VIOLET
        ax.plot(s['t'], s['g'] / s['start'], color=col, lw=1.6)
        ax.plot(s['t_trip'], s['trip'] / s['start'], 'o', color=col, ms=5, mec=SURF, mew=1.5, zorder=4)
        ax.annotate(f'{s["name"].split()[1]} x{s["trip"] / s["start"]:.1f}', (s['t_trip'], s['trip'] / s['start']),
                    xytext=(-4, 6), textcoords='offset points', ha='right', fontsize=7.5, color=INK2)
    ax.set_yscale('log', base=2); ax.set_yticks([1, 2, 4, 8]); ax.set_yticklabels(['x1', 'x2', 'x4', 'x8'])
    ax.set_ylim(0.8, 13)
    ax.plot([], [], color=BLUE, label='roll gains'); ax.plot([], [], color=VIOLET, label='pitch gains')
    ax.legend(loc='upper left', ncol=2)
    ax.set_ylabel('Gain under test\n/ starting value')
    ax.set_title('b  Gain being ramped (doubles every 10 s until the oscillation detector trips)', fontsize=9.5)
    # c) rate tracking error
    ax = axs[2]
    e = R['rate_err']
    ax.plot(e[:, 0], e[:, 1], color=BLUE, label='roll')
    ax.plot(e[:, 0], e[:, 2], color=VIOLET, label='pitch')
    ax.legend(loc='upper left', ncol=2)
    ax.set_ylim(0, 25)
    ax.set_ylabel('Rate error RMS\n(deg/s, 1 s)')
    ax.set_title('c  Rate-tracking error: quiet through the roll tune, growing through the pitch tune', fontsize=9.5)
    # d) motors
    ax = axs[3]
    motor_lines(ax, L, R['arm2'] + 3.5, R['shock1'], mean_window=1.0)
    ax.set_ylabel('Motor output\n(PWM us, 1 s mean)')
    ax.set_ylim(1350, 1950)
    ax.legend(loc='upper left', ncol=4, fontsize=7.5)
    ax.set_title('d  Motor outputs: M4 (rear-right) carries the most load all flight', fontsize=9.5)
    # e) current
    ax = axs[4]
    tb = L.t('BAT'); m = (tb >= t0) & (tb < t1)
    ax.step(tb[m], L.D['BAT']['Curr'][m], where='post', color=INK, lw=1.0)
    ax.set_ylabel('Battery\ncurrent (A)'); ax.set_xlabel('Time since boot (s)')
    ax.set_title('e  Battery current', fontsize=9.5)
    mark(axs, [(R['shock1'], 'failure'), (R['impact'], '')])
    axs[-1].set_xlim(t0, t1)
    fig.tight_layout(rect=(0, 0, 1, 0.975))
    fig.savefig(os.path.join(out, 'fig1_flight2_overview.png'), dpi=150)
    plt.close(fig)


def fig_last_seconds(L, R, out):
    plt = style()
    t0, t1 = R['stages'][-1]['t0'] - 0.5, R['impact'] + 0.3
    fig, axs = plt.subplots(6, 1, figsize=(12, 13.5), sharex=True,
                            gridspec_kw=dict(height_ratios=[1, 1, 1, 1.15, 0.9, 0.8]))
    fig.suptitle('Final Pitch P ramp: growing oscillation, M4 driven to its limit, then structural failure',
                 x=0.01, ha='left', fontsize=12.5, fontweight='bold')
    r, tr = L.D['RATE'], L.t('RATE'); m = (tr >= t0) & (tr < t1)
    for ax, (d, a, lab, col) in zip(axs[:3], [('RDes', 'R', 'Roll', BLUE), ('PDes', 'P', 'Pitch', VIOLET),
                                              ('YDes', 'Y', 'Yaw', AQUA)]):
        ax.plot(tr[m], r[d][m], color=MUTED, lw=1.0, label='demanded')
        ax.plot(tr[m], r[a][m], color=col, lw=1.0, label='measured')
        ax.set_ylim(-60, 60); ax.set_ylabel(f'{lab} rate\n(deg/s)')
        ax.legend(loc='lower left', ncol=2)
    axs[0].set_title('a  Roll rate: ~1 Hz rocking from ~300 s (roll was not being tuned); off scale after failure',
                     fontsize=9.5)
    axs[1].set_title('b  Pitch rate: 5-6 Hz oscillation grows as Pitch P passes ~0.55', fontsize=9.5)
    axs[2].set_title('c  Yaw rate: clockwise drift from 308.7 s against the demand (similar excursions occurred earlier)', fontsize=9.5)
    ax = axs[3]
    motor_lines(ax, L, t0, t1, label_end=False)
    ax.axhline(L.P['Q_M_PWM_MIN'] + L.P['Q_M_SPIN_MAX'] * (L.P['Q_M_PWM_MAX'] - L.P['Q_M_PWM_MIN']),
               color=INK2, lw=0.8)
    ax.text(t0 + 0.1, 1958, 'max output (Q_M_SPIN_MAX)', fontsize=7.5, color=INK2, va='bottom')
    ax.set_ylim(1100, 2010); ax.set_ylabel('Motor output\n(PWM us)')
    ax.legend(loc='lower left', ncol=4, fontsize=7.5, frameon=True, facecolor=SURF, edgecolor='none',
              framealpha=0.9)
    ax.set_title('d  Motor outputs (25 Hz): M4 climbs to ~1920 us in the last half-second', fontsize=9.5)
    ax = axs[4]
    tq = L.t('QTUN'); mq = (tq >= t0) & (tq < t1)
    cmd = L.D['QTUN']['ThO'][mq] / L.D['QTUN']['ThH'][mq]
    ax.plot(tq[mq], smooth(cmd, 5), color=MUTED, label='commanded thrust / hover thrust')
    tl, low = L.lift_over_weight(); ml = (tl >= t0) & (tl < t1)
    ax.plot(tl[ml], smooth(low[ml], 10), color=INK, lw=1.2, label='achieved lift / weight (EKF vertical accel.)')
    ax.set_ylim(-0.2, 2.2); ax.set_ylabel('x hover')
    ax.legend(loc='upper left', ncol=2)
    ax.set_title('e  Thrust: commanded vs achieved. From ~308 s the command climbs to 1.7x hover for ~1 g of lift',
                 fontsize=9.5)
    ax = axs[5]
    tb = L.t('BAT'); mb = (tb >= t0) & (tb < t1)
    ax.step(tb[mb], L.D['BAT']['Curr'][mb], where='post', color=INK, lw=1.0)
    ax.set_ylabel('Battery\ncurrent (A)'); ax.set_xlabel('Time since boot (s)')
    ax.set_title('f  Battery current', fontsize=9.5)
    trip = R['stages'][-1]['t_trip']
    mark(axs, [(trip, 'P trip'), (R['shock1'], 'failure'), (R['impact'], 'impact')])
    axs[-1].set_xlim(t0, t1)
    fig.tight_layout(rect=(0, 0, 1, 0.975))
    fig.savefig(os.path.join(out, 'fig2_final_pitch_ramp.png'), dpi=150)
    plt.close(fig)


def fig_failure(L, R, out):
    plt = style()
    t0, t1 = R['shock1'] - 0.3, R['impact'] + 0.35
    fig, axs = plt.subplots(5, 1, figsize=(12, 12), sharex=True,
                            gridspec_kw=dict(height_ratios=[1.2, 1, 0.9, 1, 0.8]))
    fig.suptitle('The failure: a structural shock, then all four lift motors stop drawing current',
                 x=0.01, ha='left', fontsize=12.5, fontweight='bold')
    t, a = L.imu0(); m = (t >= t0) & (t < t1)
    ax = axs[0]
    for k, col, lab in (('AccX', BLUE, 'X (forward)'), ('AccY', VIOLET, 'Y (right)'), ('AccZ', AQUA, 'Z (down)')):
        ax.plot(t[m], a[k][m], color=col, lw=0.9, label=lab)
    ax.set_ylim(-30, 30); ax.set_ylabel('Accelerometer\n(m/s^2, 300 Hz)')
    ax.legend(loc='lower left', ncol=3)
    ax.text(R['impact'] - 0.02, -27, f'impact: {R["impact_g_50ms"]:.0f} g mean over 50 ms (off scale) ',
            fontsize=7.5, color=INK2, va='bottom', ha='right')
    ax.set_title('a  Body accelerations: 60 ms shock at failure, a second shock in the air, then impact',
                 fontsize=9.5)
    ax = axs[1]
    ts, sf = L.specific_force_g(); ms = (ts >= t0) & (ts < t1)
    ax.plot(ts[ms], smooth(sf[ms], 6), color=INK, lw=1.1)
    ax.axhline(1.0, color=MUTED, lw=0.8); ax.axhline(0.0, color=MUTED, lw=0.8)
    ax.text(t0 + 0.02, 1.05, 'hover (1 g)', fontsize=7.5, color=INK2, va='bottom')
    ax.text(t0 + 0.02, 0.05, 'free fall (0 g)', fontsize=7.5, color=INK2, va='bottom')
    ax.set_ylim(-0.2, 2.5); ax.set_ylabel('Specific force\n|a| / g')
    ax.set_title('b  Accelerometer magnitude: ~1 g on the motors, ~0.1-0.3 g after the shock (near free fall), '
                 'then rising with air drag', fontsize=9.5)
    ax = axs[2]
    tb = L.t('BAT'); mb = (tb >= t0 - 0.2) & (tb < t1)
    ax.step(tb[mb], L.D['BAT']['Curr'][mb], where='post', color=INK, lw=1.2)
    ax.set_ylabel('Battery\ncurrent (A)')
    ax.set_title('c  Battery current (10 Hz): collapses from ~22 A to ~1 A; one brief return', fontsize=9.5)
    ax = axs[3]
    motor_lines(ax, L, t0, t1, label_end=False)
    ax.set_ylim(1100, 2010); ax.set_ylabel('Motor output\n(PWM us)')
    ax.legend(loc='lower left', ncol=4, fontsize=7.5, frameon=True, facecolor=SURF, edgecolor='none',
              framealpha=0.9)
    ax.set_title('d  Motor commands stay high: the autopilot keeps asking, the motors do not respond',
                 fontsize=9.5)
    ax = axs[4]
    tq = L.t('QTUN'); mq = (tq >= t0) & (tq < t1)
    ax.plot(tq[mq], L.D['QTUN']['Alt'][mq] - R['alt0'], color=INK)
    ax.set_ylabel('Height above\ntake-off (m)'); ax.set_xlabel('Time since boot (s)')
    ax.set_title(f'e  Height: ~1.5 s of near free fall from {R["h_shock"]:.1f} m', fontsize=9.5)
    mark(axs, [(R['shock1'], 'shock'), (R['abort'], 'abort'), (R['shock2'], '2nd shock'),
               (R['impact'], 'impact')])
    axs[-1].set_xlim(t0, t1)
    fig.tight_layout(rect=(0, 0, 1, 0.975))
    fig.savefig(os.path.join(out, 'fig3_failure_detail.png'), dpi=150)
    plt.close(fig)


def fig_imbalance(L, R, out):
    plt = style()
    h = R['hover']['flight 2 pre-tune']
    names = [n for n, _, _ in MOTORS]
    mean = np.mean([h['thr'][n] for n in names])
    vals = [100 * h['thr'][n] / mean for n in names]
    fig, ax = plt.subplots(figsize=(8, 4.6))
    fig.suptitle('Steady hover before tuning: M4 commanded ~2x the thrust of M1', x=0.01, ha='left',
                 fontsize=12, fontweight='bold')
    xs = np.arange(4)
    bars = ax.bar(xs, vals, width=0.45, color=[MGRAY[n] if n != 'M4' else ORANGE for n in names], zorder=3)
    ax.axhline(100, color=INK2, lw=0.8, zorder=2)
    for x, v, n in zip(xs, vals, names):
        ax.text(x, v + 2, f'{v:.0f}%\n{h["pwm"][n]:.0f} us', ha='center', va='bottom', fontsize=8.5, color=INK)
    ax.set_xticks(xs)
    ax.set_xticklabels([f'{n}\n{d}' for n, _, d in MOTORS], fontsize=8.5, color=INK2)
    ax.set_ylabel('Commanded thrust\n(% of four-motor mean)')
    ax.set_ylim(0, 175)
    ax.grid(axis='x', visible=False)
    fmt = lambda v: f'{v:+.2f}' if abs(v) >= 0.005 else '0.00'
    ax.text(0.01, 0.97, f'Mean controller trims in hover: pitch {fmt(h["POut"])} (rear motors high), '
                        f'yaw {fmt(h["YOut"])} (CW motors high), roll {fmt(h["ROut"])}',
            transform=ax.transAxes, ha='left', va='top', fontsize=8, color=INK2)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(os.path.join(out, 'fig4_hover_imbalance.png'), dpi=150)
    plt.close(fig)


def fig_causality(L, R, out):
    plt = style()
    C = R['causal']
    ends, brk = C['ends'], R['shock1']
    t0, t1 = R['stages'][-1]['t0'] - 1.5, brk + 0.12
    fig, axs = plt.subplots(5, 1, figsize=(12, 12), sharex=True,
                            gridspec_kw=dict(height_ratios=[0.8, 1.1, 1.0, 0.9, 0.9]))
    fig.suptitle('Which came first? The oscillation built for ~10 s before any mechanical sign appeared',
                 x=0.01, ha='left', fontsize=12.5, fontweight='bold')
    trip = R['stages'][-1]['t_trip']
    for ax in axs:
        ax.axvspan(R['stages'][-1]['t0'], brk - 0.3, color='#efeee9', lw=0, zorder=0)
        ax.axvspan(brk - 0.3, brk, color='#f7e4dc', lw=0, zorder=0)
    axs[0].text((R['stages'][-1]['t0'] + brk - 0.3) / 2, 0.93, 'oscillating, vibration normal',
                transform=axs[0].get_xaxis_transform(), ha='center', va='top', fontsize=8, color=INK2)
    # a) gain
    ax = axs[0]
    s_ = R['stages'][-1]
    ax.plot(s_['t'], s_['g'], color=VIOLET, lw=1.6)
    ax.set_ylabel('Pitch P\nunder test'); ax.set_ylim(0.2, 0.72)
    ax.set_title('a  QuickTune ramps Pitch P (and I); it trips at 308.93 s and cuts the gain', fontsize=9.5)
    # b) oscillation amplitude
    ax = axs[1]
    k = ends >= t0
    ax.plot(ends[k], C['pitch'][k], color=VIOLET, label='pitch 4-8 Hz oscillation')
    ax.plot(ends[k], C['roll'][k], color=BLUE, label='roll 1-2.5 Hz rocking')
    ax.set_ylabel('Oscillation\n(deg/s RMS, 0.4 s)')
    ax.legend(loc='upper left', ncol=2)
    ax.set_title('b  Oscillation grows with the gain; the pitch mode peaks and starts to shrink once the gain is cut',
                 fontsize=9.5)
    # c) M4 load
    ax = axs[2]
    to = L.t('RCOU'); mo = (to >= t0) & (to < brk)
    ax.plot(to[mo], L.D['RCOU']['C6'][mo], color=ORANGE, lw=1.6)
    ax.axhline(L.P['Q_M_PWM_MIN'] + L.P['Q_M_SPIN_MAX'] * (L.P['Q_M_PWM_MAX'] - L.P['Q_M_PWM_MIN']),
               color=INK2, lw=0.8)
    ax.text(t0 + 0.1, 1955, 'max output', fontsize=7.5, color=INK2, va='bottom')
    ax.set_ylim(1500, 2000); ax.set_ylabel('M4 command\n(PWM us)')
    ax.set_title('c  Load on M4 (rear-right) cycles harder and climbs to its flight maximum at the break', fontsize=9.5)
    # d) vibration
    ax = axs[3]
    ax.plot(ends[k], C['vib'][k], color=INK, lw=1.2)
    ax.axhline(C['vib_thr'], color=MUTED, lw=0.8)
    ax.text(t0 + 0.1, C['vib_thr'] * 1.05, 'normal range (99.9th pct)', fontsize=7.5, color=INK2, va='bottom')
    ax.set_ylim(0, 1.0); ax.set_ylabel('Vibration >15 Hz\n(m/s^2 RMS, 0.1 s)')
    ax.set_title('d  Structural vibration stays in its normal range until the last ~0.25 s', fontsize=9.5)
    # e) efficiency
    ax = axs[4]
    ax.plot(ends[k], C['eff'][k], color=INK, lw=1.2)
    ax.axhline(C['eff_prev_min'], color=MUTED, lw=0.8)
    ax.text(t0 + 0.1, C['eff_prev_min'] - 0.03, 'lowest earlier in the ramp', fontsize=7.5, color=INK2, va='top')
    ax.set_ylim(0.5, 1.2); ax.set_ylabel('Lift / commanded\n(0.5 s)')
    ax.set_xlabel('Time since boot (s)')
    ax.set_title('e  Lift per unit of commanded thrust: dips during rocking recover; the final slide does not',
                 fontsize=9.5)
    mark(axs, [(trip, 'P trip'), (brk, 'break')])
    axs[-1].set_xlim(t0, t1)
    fig.text(0.01, 0.005, 'Causal filters and trailing windows, plotted at the window end: each point uses only '
             'past data.\nThis delays the oscillation curve (~0.25 s) more than the vibration curve (~0.05 s), so '
             'the plot understates how early the oscillation came.', fontsize=7.5, color=INK2, ha='left', va='bottom')
    fig.tight_layout(rect=(0, 0.025, 1, 0.975))
    fig.savefig(os.path.join(out, 'fig5_which_came_first.png'), dpi=150)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('log', help='DataFlash .BIN log')
    ap.add_argument('--out', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'figures'))
    args = ap.parse_args()
    L = Log(*load(args.log))
    R = analyse(L)
    summary(L, R)
    precursors(L, R)
    causality(L, R)
    weathervane(L, R)
    os.makedirs(args.out, exist_ok=True)
    fig_overview(L, R, args.out)
    fig_last_seconds(L, R, args.out)
    fig_failure(L, R, args.out)
    fig_imbalance(L, R, args.out)
    fig_causality(L, R, args.out)
    print(f'\nFigures written to {args.out}')


if __name__ == '__main__':
    main()
