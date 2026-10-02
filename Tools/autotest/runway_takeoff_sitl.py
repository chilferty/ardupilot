#!/usr/bin/env python3
"""
SITL scenarios for the runway centreline takeoff (RWY_*) feature.

Starts arduplane SITL directly (no MAVProxy), drives it over MAVLink with
pymavlink and reports the cross-track error during the takeoff roll.

usage:
  python3 Tools/autotest/runway_takeoff_sitl.py [scenario ...]

scenarios: stock, track, refuse, abort, past_v1, all (default)

The runway is synthetic: heading RWY_HDG_DEG true, reference point
REF_DIST_M ahead of a "threshold" point near CMAC. The aircraft is placed
START_OFFSET_M to the right of the centreline, yawed START_YAW_ERR_DEG
off the runway heading, with a crosswind, to make the stock heading-hold
drift.
"""

import math
import os
import subprocess
import sys
import time

from pymavlink import mavutil

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
BINARY = os.path.join(ROOT, "build", "sitl", "bin", "arduplane")

THRESH_LAT = -35.363261
THRESH_LON = 149.165230
THRESH_ALT = 584
RWY_HDG_DEG = 90.0
REF_DIST_M = 1500.0

EARTH_R = 6378137.0


def offset(lat, lon, bearing_deg, dist_m):
    b = math.radians(bearing_deg)
    dn = math.cos(b) * dist_m
    de = math.sin(b) * dist_m
    lat2 = lat + math.degrees(dn / EARTH_R)
    lon2 = lon + math.degrees(de / (EARTH_R * math.cos(math.radians(lat))))
    return lat2, lon2


REF_LAT, REF_LON = offset(THRESH_LAT, THRESH_LON, RWY_HDG_DEG, REF_DIST_M)


def line_errors(lat, lon):
    """xtrk (+ve right) and along-track (m, relative to threshold)"""
    dn = math.radians(lat - THRESH_LAT) * EARTH_R
    de = math.radians(lon - THRESH_LON) * EARTH_R * math.cos(math.radians(THRESH_LAT))
    h = math.radians(RWY_HDG_DEG)
    along = dn * math.cos(h) + de * math.sin(h)
    xtrk = math.cos(h) * de - math.sin(h) * dn
    return xtrk, along


BASE_PARAMS = {
    # ground steering on SERVO5 for the plane-ice-steering SITL frame
    "SERVO5_FUNCTION": 26,
    "GROUND_STEER_ALT": 5,
    "STEER2SRV_P": 3.0,
    "STEER2SRV_I": 0.0,
    "STEER2SRV_D": 0.0,
    "STEER2SRV_MINSPD": 2,
    # takeoff
    "TKOFF_ROTATE_SPD": 15,
    "TKOFF_THR_MAX": 45,   # slow acceleration -> long ground roll
    "TKOFF_THR_MINSPD": 0,
    "TKOFF_THR_MINACC": 0,
    "TKOFF_THR_DELAY": 0,
    "TKOFF_GND_PITCH": 0,
    "LEVEL_ROLL_LIMIT": 5,
    # brakes on SERVO9 as RC9 passthrough
    "SERVO9_FUNCTION": 59,
    # crosswind from the right
    "SIM_WIND_SPD": 6,
    "SIM_WIND_DIR": 180,
    "SIM_WIND_TURB": 1,
    "LOG_DISARMED": 0,
    "RC_OVERRIDE_TIME": -1,
    "RC11_OPTION": 0,         # no engine RC switch (engine by MAVLink command)  # overrides never time out (test determinism)
}

RWY_PARAMS = {
    "RWY_ENABLE": 1,
    "RWY_HDG": RWY_HDG_DEG,
    "RWY_STRT_XTRK": 3,
    "RWY_STRT_HDG": 5,
    "RWY_STRT_DIST": 1000,
    "RWY_ABT_XTRK": 0,
    "RWY_ABT_HDG": 0,
    "RWY_V1": 0,
    "RWY_BRK_CHAN": 9,
    "RWY_BRK_PWM": 1900,
}


class Sim:
    def __init__(self, start_offset_m, start_yaw_err_deg, instance=0):
        lat, lon = offset(THRESH_LAT, THRESH_LON, RWY_HDG_DEG + 90, start_offset_m)
        yaw = (RWY_HDG_DEG + start_yaw_err_deg) % 360
        home = "%.8f,%.8f,%.1f,%.1f" % (lat, lon, THRESH_ALT, yaw)
        self.workdir = "/tmp/rwy_sitl_%d" % instance
        os.makedirs(self.workdir, exist_ok=True)
        for f in os.listdir(self.workdir):
            if f.endswith(".bin") or f == "eeprom.bin":
                os.unlink(os.path.join(self.workdir, f))
        defaults = ",".join([
            os.path.join(ROOT, "Tools/autotest/models/plane.parm"),
            os.path.join(ROOT, "Tools/autotest/default_params/plane-ice.parm"),
            # sets RC11_OPTION 0 at boot: engine by MAVLink command only
            os.path.join(ROOT, "Tools/autotest/runway_takeoff_sitl.parm"),
        ])
        cmd = [BINARY, "--model", "plane-ice-steering", "--home", home,
               "--defaults", defaults, "--speedup", os.environ.get("RWY_SPEEDUP", "5"), "-I%d" % instance, "-w"]
        self.proc = subprocess.Popen(cmd, cwd=self.workdir,
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(2)
        self.mav = mavutil.mavlink_connection("tcp:127.0.0.1:%d" % (5760 + 10 * instance),
                                              source_system=255)
        self.mav.wait_heartbeat(timeout=30)
        self.texts = []
        self.track = []      # (t, xtrk, along, spd, alt)
        self.brake_pwm = []  # (t, pwm9, pwm3)
        self.last_rc = {}
        for stream in (mavutil.mavlink.MAV_DATA_STREAM_ALL,):
            self.mav.mav.request_data_stream_send(1, 1, stream, 20, 1)

    def close(self):
        try:
            self.proc.terminate()
            self.proc.wait(5)
        except Exception:
            self.proc.kill()

    def pump(self, duration=0.0):
        end = time.time() + duration
        while True:
            m = self.mav.recv_match(blocking=True, timeout=0.05)
            if m is not None:
                self.handle(m)
            if time.time() >= end:
                break

    def handle(self, m):
        t = m.get_type()
        if t == "STATUSTEXT":
            self.texts.append(m.text)
            print("    [GCS] %s" % m.text)
        elif t == "GLOBAL_POSITION_INT":
            x, a = line_errors(m.lat * 1e-7, m.lon * 1e-7)
            spd = math.hypot(m.vx, m.vy) * 0.01
            self.track.append((time.time(), x, a, spd, m.relative_alt * 0.001))
        elif t == "SERVO_OUTPUT_RAW":
            self.brake_pwm.append((time.time(), m.servo9_raw, m.servo3_raw))

    def set_param(self, name, value):
        for _ in range(5):
            self.mav.param_set_send(name, float(value))
            t0 = time.time()
            while time.time() - t0 < 1.5:
                m = self.mav.recv_match(blocking=True, timeout=0.1)
                if m is None:
                    continue
                if m.get_type() == "PARAM_VALUE" and m.param_id == name:
                    if abs(m.param_value - float(value)) < 1e-3:
                        return
                else:
                    self.handle(m)
        raise RuntimeError("failed to set %s" % name)

    def set_params(self, d):
        for k, v in d.items():
            self.set_param(k, v)

    def rc_override(self, chans):
        self.last_rc.update(chans)
        vals = [65535] * 18
        for ch, pwm in self.last_rc.items():
            vals[ch - 1] = pwm
        self.mav.mav.rc_channels_override_send(1, 1, *vals)

    def upload_mission(self, ref=True):
        items = []
        # 0: home
        items.append((mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, THRESH_LAT, THRESH_LON, 0,
                      mavutil.mavlink.MAV_FRAME_GLOBAL))
        # 1: takeoff, lat/lon = runway reference point
        items.append((mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 10, 0, 0, 0,
                      REF_LAT if ref else 0, REF_LON if ref else 0, 40,
                      mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT))
        far_lat, far_lon = offset(THRESH_LAT, THRESH_LON, RWY_HDG_DEG, 4000)
        items.append((mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, far_lat, far_lon, 80,
                      mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT))
        items.append((mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM, 0, 0, 0, 0, far_lat, far_lon, 80,
                      mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT))
        self.mav.mav.mission_count_send(1, 1, len(items))
        sent = set()
        t0 = time.time()
        while time.time() - t0 < 20:
            m = self.mav.recv_match(type=["MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK"],
                                    blocking=True, timeout=1)
            if m is None:
                continue
            if m.get_type() == "MISSION_ACK":
                if m.type != 0:
                    raise RuntimeError("mission ack %d" % m.type)
                return
            cmd, p1, p2, p3, p4, lat, lon, alt, frame = items[m.seq]
            self.mav.mav.mission_item_int_send(1, 1, m.seq, frame, cmd, 0, 1, p1, p2, p3, p4,
                                               int(lat * 1e7), int(lon * 1e7), alt,
                                               mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            sent.add(m.seq)
        raise RuntimeError("mission upload timeout")

    def wait_ready(self):
        # wait for EKF to be using GPS
        t0 = time.time()
        while time.time() - t0 < 120:
            m = self.mav.recv_match(type=["EKF_STATUS_REPORT", "STATUSTEXT", "GPS_RAW_INT"],
                                    blocking=True, timeout=1)
            if m is None:
                continue
            if m.get_type() == "STATUSTEXT":
                self.handle(m)
            if m.get_type() == "EKF_STATUS_REPORT" and (m.flags & 0x10) and (m.flags & 0x08) \
                    and time.time() - t0 > 20:
                return
        raise RuntimeError("EKF not ready")

    def set_mode(self, name):
        mode_id = self.mav.mode_mapping()[name]
        self.mav.set_mode(mode_id)
        self.pump(1)

    def arm(self):
        self.mav.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                       1, 2989, 0, 0, 0, 0, 0)
        self.pump(1.5)

    def disarm(self):
        self.mav.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                       0, 21196, 0, 0, 0, 0, 0)
        self.pump(1.5)

    def start_engine(self):
        # no engine RC switch: start with MAV_CMD_DO_ENGINE_CONTROL,
        # allowing a start while disarmed (flags bit 0)
        self.rc_override({9: 1100, 3: 1000})
        self.mav.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_DO_ENGINE_CONTROL, 0,
                                       1, 0, 0, 1, 0, 0, 0)
        self.pump(8)

    def run_takeoff(self, seconds):
        t_end = time.time() + seconds
        while time.time() < t_end:
            self.rc_override({})   # keep overrides alive
            self.pump(0.5)

    def roll_stats(self, max_alt=1.0):
        pts = [p for p in self.track if p[3] > 1.0 and p[4] < max_alt]
        if not pts:
            return None
        xs = [p[1] for p in pts]
        return {
            "n": len(pts),
            "start_xt": xs[0],
            "end_xt": xs[-1],
            "max_abs_xt": max(abs(x) for x in xs),
            "max_spd": max(p[3] for p in pts),
            "dist": pts[-1][2] - pts[0][2],
        }


def scenario(name, params, start_offset, yaw_err, run_s, extra=None, ref=True):
    print("\n=== %s: start offset %.1fm right, yaw err %+.1f deg ===" % (name, start_offset, yaw_err))
    sim = Sim(start_offset, yaw_err)
    try:
        sim.set_params(BASE_PARAMS)
        sim.set_params(params)
        if os.environ.get("RWY_EXTRA_PARAMS"):
            import json
            sim.set_params(json.loads(os.environ["RWY_EXTRA_PARAMS"]))
        sim.wait_ready()
        for attempt in range(3):
            try:
                sim.upload_mission(ref=ref)
                break
            except RuntimeError as ex:
                print("  mission upload retry: %s" % ex)
                sim.pump(2)
        sim.set_mode("AUTO")
        sim.arm()
        # SITL only drives the starter when armed, so start after arming
        sim.start_engine()
        if extra is not None:
            extra(sim)
        else:
            sim.run_takeoff(run_s)
        st = sim.roll_stats()
        air = [p for p in sim.track if p[4] > 30]
        res = {"stats": st, "texts": list(sim.texts),
               "airborne_xt": air[0][1] if air else None,
               "final_alt": sim.track[-1][4] if sim.track else None,
               "brake": list(sim.brake_pwm)}
        print("  roll stats: %s" % st)
        # cross-track profile against distance down the runway
        prof = []
        nxt = 0
        for (_, x, a, spd, alt) in sim.track:
            if a >= nxt and spd > 0.5:
                prof.append("%4.0fm:%+5.1f%s" % (a, x, "*" if alt > 1 else ""))
                nxt += 50
            if a > 1200:
                break
        print("  xt profile (along:xt, *=airborne): " + " ".join(prof))
        if air:
            print("  xt when passing 30m AGL: %.1fm" % air[0][1])
        return res
    finally:
        sim.close()


def abort_extra(sim):
    # run until abort, check engine/brakes, then check the engine stays
    # stopped and the abort clears on disarm
    t0 = time.time()
    while time.time() - t0 < 60 and not any("ABORT" in t for t in sim.texts):
        sim.rc_override({})
        sim.pump(0.3)
    sim.run_takeoff(12)
    print("  last SERVO3 (thr) %s SERVO9 (brake) %s" %
          (sim.brake_pwm[-1][2], sim.brake_pwm[-1][1]))
    # a start command while the abort is latched must not start the engine
    sim.mav.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_DO_ENGINE_CONTROL, 0,
                                  1, 0, 0, 0, 0, 0, 0)
    sim.run_takeoff(6)
    sim.disarm()
    for _ in range(8):
        sim.rc_override({})
        sim.pump(0.5)
    print("  after disarm SERVO9 %s" % sim.brake_pwm[-1][1])


def main():
    want = sys.argv[1:] or ["all"]
    results = {}

    def on(n):
        return "all" in want or n in want

    if on("stock"):
        results["stock"] = scenario("stock (RWY_ENABLE=0)", {"RWY_ENABLE": 0}, 2.0, 3.0, 45)
    if on("track"):
        results["track"] = scenario("runway tracking", RWY_PARAMS, 2.0, 3.0, 45)
    if on("refuse"):
        p = dict(RWY_PARAMS)
        results["refuse"] = scenario("start check refuses (5m off)", p, 5.0, 0.0, 15)
    if on("nodef"):
        results["nodef"] = scenario("no runway point in TAKEOFF item", RWY_PARAMS, 0.0, 0.0, 15, ref=False)
    if on("abort"):
        # steering removed so the crosswind/yaw error pushes it off the line
        p = dict(RWY_PARAMS, RWY_ABT_XTRK=3, RWY_V1=13, SERVO5_FUNCTION=0, KFF_RDDRMIX=0,
                 RWY_STRT_HDG=5)
        results["abort"] = scenario("abort below V1 (no steering)", p, 0.0, 3.0, 0, extra=abort_extra)
    if on("past_v1"):
        p = dict(RWY_PARAMS, RWY_ABT_XTRK=3, RWY_V1=8, SERVO5_FUNCTION=0, KFF_RDDRMIX=0)
        results["past_v1"] = scenario("deviation after V1 continues", p, 0.0, 3.0, 45)

    print("\n=== SUMMARY ===")
    for k, r in results.items():
        st = r["stats"]
        aborted = any("ABORT" in t for t in r["texts"])
        refused = any("RWY: start" in t or "RWY: no runway" in t for t in r["texts"])
        print("%-8s max|xt| roll %s  xt@30m %s  final alt %s  aborted=%s refused=%s" % (
            k,
            "%.2f" % st["max_abs_xt"] if st else "-",
            "%.2f" % r["airborne_xt"] if r["airborne_xt"] is not None else "-",
            "%.0f" % r["final_alt"] if r["final_alt"] is not None else "-",
            aborted, refused))


if __name__ == "__main__":
    main()
