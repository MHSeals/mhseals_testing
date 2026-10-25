"""Write the data of the tuning document's figures from the frozen Python-plant study rows.

Run from any directory:  python3 -I docs/figures/tuning/make_all.py

Input:  study/{plant_v1,disturb_v1,noise_v1,rules_v1,stale_v1}.jsonl, one row per
        (case, seed), frozen from the crane_tuning plant study with the characterization
        (``hw.characterization``) dropped; a case that changed the velocity-loop gains keeps
        them as ``controller`` (``hw.controller`` in the original row).
Output: data/*.dat (whitespace columns with a header line, '#' comments, read by
        \\addplot table) and data/summary.txt (every number a caption quotes, with its
        case and seeds). Standard library only; deterministic.
"""
import json
import os
import statistics

HERE = os.path.dirname(os.path.abspath(__file__))
STUDY = os.path.join(HERE, "study")
OUT = os.path.join(HERE, "data")

SUMMARY = []


def load(name):
    with open(os.path.join(STUDY, name + ".jsonl")) as f:
        return [dict(json.loads(line), study=name) for line in f if line.strip()]


def write(name, columns, rows, header=""):
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, name + ".dat"), "w") as f:
        for line in header.strip().splitlines():
            f.write("# " + line + "\n")
        f.write(" ".join(columns) + "\n")
        for row in rows:
            f.write(" ".join(f"{float(v):.6g}" for v in row) + "\n")


def note(text):
    SUMMARY.append(text)


def stat(values):
    """(mean, min, max): the point is the seed mean (one value for a single seed)."""
    return statistics.fmean(values), min(values), max(values)


def how(rows):
    return "seed 0" if len(rows) == 1 else f"mean of seeds {seeds(rows)}"


def seeds(rows):
    return ",".join(str(r["seed"]) for r in sorted(rows, key=lambda r: r["seed"]))


def tag(rows):
    """'study:case[seeds]' for every case among ``rows``."""
    by = {}
    for r in rows:
        by.setdefault((r["study"], r["case"]), []).append(r)
    return " + ".join(f"{s}:{c}[s{seeds(rs)}]" for (s, c), rs in sorted(by.items()))


# --- fig_delay: added delay (dead time + odometry latency) ------------------------------
def delay(plant, disturb):
    """Rows that differ from plant_v1:nominal only by dead_time or odom_latency."""
    nominal = next(r for r in plant if r["case"] == "nominal")
    keyed = ("name", "dead_time", "odom_latency")
    base = {k: v for k, v in nominal["variant"].items() if k not in keyed}

    def only_delay(r):
        v = r["variant"]
        if v["dead_time"] + v["odom_latency"] == 0:
            return r is nominal  # zero delay is plant_v1:nominal itself (disturb_v1:calm is identical)
        return r["nav"] == nominal["nav"] and "controller" not in r and all(
            v.get(k) == x for k, x in base.items())

    rows = [r for r in plant + disturb if only_delay(r)]
    note("fig_delay: rows that differ from plant_v1:nominal only by dead_time / odom_latency")
    note("  " + tag(rows))
    for kind, key, other in (("dead", "dead_time", "odom_latency"), ("latency", "odom_latency", "dead_time")):
        sel = [r for r in rows if r["variant"][other] == 0]
        for metric, get, scale in (("overshoot", lambda r: r["step"]["yaw"]["overshoot"], 100),
                                   ("yawrev", lambda r: r["route"]["yaw_rev_per_min"], 1)):
            points = {}
            for r in sel:
                if metric == "overshoot" and "step" not in r:
                    continue
                points.setdefault(r["variant"][key], []).append(r)
            table = []
            for d in sorted(points):
                val, lo, hi = stat([get(r) * scale for r in points[d]])
                table.append((d, val, val - lo, hi - val))
                note(f"  {kind:7s} {metric:9s} {key} {d:.2f} s: {val:.4g} ({how(points[d])}"
                     + ("" if len(points[d]) == 1 else f", min {lo:.4g}, max {hi:.4g}")
                     + f") <- {tag(points[d])}")
            write(f"delay_{metric}_{kind}", ("delay", "val", "lo", "hi"), table,
                  f"{metric} vs added {key} (s); seed mean (one value for seed 0), lo/hi = distance to min/max\n"
                  f"rows: {tag([r for p in points.values() for r in p])}")
    n_os = sorted({r["variant"]["dead_time"] + r["variant"]["odom_latency"] for r in rows if "step" in r})
    n_rev = sorted({r["variant"]["dead_time"] + r["variant"]["odom_latency"] for r in rows})
    note(f"  delay points: {len(n_rev)} with route data {n_rev}, {len(n_os)} with step data {n_os}; "
         "disturb_v1 rows carry no step block, plant_v1 has seed 0 only, and the disturb_v1 "
         "delay cases give identical seeds (no random input)")


# --- fig_noise: velocity kp on the field mix ---------------------------------------------
KP0 = 0.8  # surge kp of the study's default controller (kp = 0.8, 0.8, 0.6)


def noise(noise_rows, disturb):
    by = {}
    for r in noise_rows:
        by.setdefault(r["case"], []).append(r)
    series = {"kp": ["kp0_ff_only", "kp0.25", "kp0.5", "kp1.0_ki1.0"],
              "kyaw": ["kyaw0.5_kp0.5", "kyaw0.5"]}
    note("fig_noise: noise_v1 field mix (dead_time 0.05, odom_latency 0.1, vel_sd 0.04, wave_sd 0.06)")
    for name, cases in series.items():
        table = []
        for case in cases:
            rs = by[case]
            kp = rs[0].get("controller", {}).get("kp", [KP0])[0] / KP0
            thr = stat([r["route"]["thr_rev_per_min"] for r in rs])
            yaw = stat([r["route"]["yaw_rev_per_min"] for r in rs])
            med = statistics.median(r["route"]["thr_rev_per_min"] for r in rs)
            table.append((kp, thr[0], thr[0] - thr[1], thr[2] - thr[0], yaw[0], yaw[0] - yaw[1], yaw[2] - yaw[0]))
            note(f"  {case:14s} kp x{kp:.2f} k_yaw {rs[0]['nav']['k_yaw']}: thruster rev/min {thr[0]:.4g} "
                 f"({how(rs)}; min {thr[1]:.4g}, max {thr[2]:.4g}, median {med:.4g}); yaw rev/min {yaw[0]:.4g} "
                 f"(min {yaw[1]:.4g}, max {yaw[2]:.4g}) <- {tag(rs)}")
        write(f"noise_{name}", ("kp", "thr", "thrlo", "thrhi", "yaw", "yawlo", "yawhi"), table,
              "route reversals/min vs velocity kp factor (x 0.8/0.8/0.6); mean of seeds, lo/hi to min/max\n"
              f"rows: {tag([r for c in cases for r in by[c]])}")
    calm = [r for r in disturb if r["case"] == "calm"]
    thr = statistics.fmean(r["route"]["thr_rev_per_min"] for r in calm)
    yaw = statistics.fmean(r["route"]["yaw_rev_per_min"] for r in calm)
    write("noise_calm", ("kp", "thr", "yaw"), [(-0.1, thr, yaw), (1.1, thr, yaw)],
          f"calm reference (no noise, no delay), kp x1: {tag(calm)}")
    note(f"  calm reference ({how(calm)}) thruster rev/min {thr:.4g}, yaw rev/min {yaw:.4g} <- {tag(calm)}")
    sigma, de = 0.04, 0.0625
    note(f"  rule kp <= de/(2 sigma_f): de {de}, sigma_f = vel_sd {sigma} m/s -> {de / (2 * sigma):.3g} "
         f"(default surge kp {KP0})")


# --- fig_limits: rule caps vs lag ----------------------------------------------------------
RULES = {"delay_s": 0.15, "tau_s": {"surge": 0.35, "sway": 0.45, "yaw": 0.15},
         "stop_distance_m": 1.0, "ramp_lags": 3.0, "yaw_tolerance": 0.25}  # characterize.yaml nav2_rules


def limits():
    k, D, tol = RULES["ramp_lags"], RULES["stop_distance_m"], RULES["yaw_tolerance"]
    ts = [0.2 + 0.01 * i for i in range(81)]
    write("limits_caps", ("T", "v", "wz"), [(t, D / (t * (k / 2 + 1)), tol / (2 * t)) for t in ts],
          f"v_cap = D/(T(k/2+1)), wz_cap = tol/(2T); D {D} m, k {k}, tol {tol} rad")
    marks = []
    note(f"fig_limits: nav2_rules (config/characterize.yaml, branch tuning-testing) D {D} m, k {k}, tol {tol} rad")
    for i, axis in enumerate(("surge", "sway", "yaw")):
        t = RULES["tau_s"][axis] + RULES["delay_s"]
        cap = tol / (2 * t) if axis == "yaw" else D / (t * (k / 2 + 1))
        marks.append((i, t, cap))
        note(f"  {axis}: T = {RULES['tau_s'][axis]} + {RULES['delay_s']} = {t:.2f} s -> cap {cap:.4g}")
    write("limits_marks", ("axis", "T", "cap"), marks, "axis 0 surge, 1 sway (m/s), 2 yaw (rad/s)")


# --- fig_spin: centre-of-mass offset rows ------------------------------------------------
def spin(plant):
    note("fig_spin: plant_v1 turn/route rows (no row records the base_link walk of an in-place turn)")
    for case in ("nominal", "com_aft10", "com_port5"):
        r = next(r for r in plant if r["case"] == case)
        note(f"  {case:10s} com {r['variant']['com']}: turn.skid_rms {r['turn']['skid_rms']}, "
             f"route yaw rev/min {r['route']['yaw_rev_per_min']}, cmd_wz rev/min {r['route']['cmd_wz_rev_per_min']} "
             f"<- plant_v1:{case}[s{r['seed']}]")
    note("  not a study row: the in-place-turn walk of base_link with com (0.10, 0) "
         "(0.156 m at wz 0.3 rad/s, 0.25 m at 0.8 rad/s) comes from ad hoc Python-plant runs "
         "(/tmp/tune/t3.py, t5.py; numpy, not frozen here)")


def main():
    plant, disturb = load("plant_v1"), load("disturb_v1")
    delay(plant, disturb)
    noise(load("noise_v1"), disturb)
    limits()
    spin(plant)
    with open(os.path.join(OUT, "summary.txt"), "w") as f:
        f.write("# Numbers the tuning figures quote; regenerate with make_all.py\n")
        f.write("\n".join(SUMMARY) + "\n")
    print("\n".join(SUMMARY))


if __name__ == "__main__":
    main()
