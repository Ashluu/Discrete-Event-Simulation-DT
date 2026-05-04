import streamlit as st
import numpy as np
import pandas as pd
import plotly.graph_objects as go
import plotly.express as px
from plotly.subplots import make_subplots
import heapq, random, math, json
from dataclasses import dataclass
from typing import List, Dict

st.set_page_config(
    page_title="CNC Digital Twin",
    page_icon="🏭",
    layout="wide",
    initial_sidebar_state="expanded",
    menu_items={
        "Get Help": "https://github.com",
        "About": "CNC Production Line Digital Twin — Discrete Event Simulation (DES)\n\nBuilt with Streamlit + Plotly. No MQTT backend needed for this demo.",
    }
)

st.markdown("""
<style>
[data-testid="stMetricValue"] { font-size: 1.6rem !important; }
.stAlert { border-radius: 8px; }
.block-container { padding-top: 1rem !important; }
</style>
""", unsafe_allow_html=True)

# ─────────────────────────────────────────────
# MACHINE DEFINITIONS
# ─────────────────────────────────────────────
BASE_DEFS = [
    dict(id=0,  name="CNC-A1", line="A", ct=8,  br=6,  rt=20, ew=6,  desc="Rough milling"),
    dict(id=1,  name="CNC-A2", line="A", ct=11, br=10, rt=28, ew=5,  desc="Finish turning"),
    dict(id=2,  name="CNC-A3", line="A", ct=7,  br=5,  rt=15, ew=7,  desc="Drilling"),
    dict(id=3,  name="CNC-A4", line="A", ct=15, br=9,  rt=35, ew=4,  desc="Grinding"),
    dict(id=4,  name="CNC-B1", line="B", ct=9,  br=7,  rt=22, ew=6,  desc="Sheet forming"),
    dict(id=5,  name="CNC-B2", line="B", ct=6,  br=4,  rt=15, ew=8,  desc="Laser cut"),
    dict(id=6,  name="CNC-B3", line="B", ct=13, br=14, rt=40, ew=5,  desc="Welding"),
    dict(id=7,  name="CNC-B4", line="B", ct=8,  br=6,  rt=18, ew=6,  desc="Deburring"),
    dict(id=8,  name="CNC-C1", line="C", ct=10, br=8,  rt=25, ew=7,  desc="Casting prep"),
    dict(id=9,  name="CNC-C2", line="C", ct=7,  br=5,  rt=12, ew=9,  desc="EDM"),
    dict(id=10, name="CNC-C3", line="C", ct=16, br=11, rt=45, ew=4,  desc="Honing"),
    dict(id=11, name="CNC-C4", line="C", ct=9,  br=7,  rt=20, ew=6,  desc="CMM inspect"),
]

LINE_COLORS = {"A": "#2563EB", "B": "#C2410C", "C": "#7C3AED"}
LINE_BG     = {"A": "#dbeafe", "B": "#fee2d5", "C": "#ede9fe"}
LINE_TC     = {"A": "#1e3a8a", "B": "#7c2d12", "C": "#4c1d95"}

# ─────────────────────────────────────────────
# MATH
# ─────────────────────────────────────────────
def rand_exp(mean):  return -mean * math.log(max(1 - random.random(), 1e-9))
def rand_norm(mu, s): return random.gauss(mu, s)
def clamp(v, a, b):  return max(a, min(b, v))

# ─────────────────────────────────────────────
# DES ENGINE
# ─────────────────────────────────────────────
class DESEngine:
    def __init__(self, mp: List[Dict], gp: Dict):
        self.mp = mp
        self.gp = gp
        self.reset()

    def reset(self):
        self.pq: list = []
        self.now = 0.0
        self.pid = 0
        self.final_out = 0
        self.line_out = {"A": 0, "B": 0, "C": 0}
        self.energy = 0.0
        self.batches = 0
        self.transit: set = set()
        self.qA = self.qB = self.qC = self.qF = 0
        self.busy = {"A": False, "B": False, "C": False, "F": False}
        self.event_log: List[Dict] = []
        self.ts = dict(t=[], final=[], wip=[], oee=[], en=[], qA=[], qB=[], qC=[], qF=[])
        self.machines = [{
            "id": d["id"], "name": d["name"], "line": d["line"],
            "state": "idle", "ss": 0.0,
            "parts_run": 0, "run_time": 0.0, "broken_time": 0.0,
            "tool_wear": 0.0, "util": 0.0,
        } for d in self.mp]
        for m in self.machines:
            heapq.heappush(self.pq, (random.uniform(0, 2), "START", m["id"], None, None))
        heapq.heappush(self.pq, (self.gp["batch"] * 24 * 60, "BATCH", 1, None, None))

    def run(self, sim_min: float, snap_count: int = 150):
        target = self.now + sim_min
        si = sim_min / snap_count
        ns = self.now + si
        while self.pq and self.pq[0][0] <= target:
            t, et, a, b, c = heapq.heappop(self.pq)
            self.now = t
            self._handle(et, a, b, c)
            if self.now >= ns:
                self._snap(); ns += si
        self._snap()

    def _handle(self, et, a, b, c):
        if et == "START":   self._start(a)
        elif et == "DONE":  self._done(a, b)
        elif et == "ARV":   self._arrive(b, c)
        elif et == "ADONE": self._adone(a)
        elif et == "ARVF":  self._arvF()
        elif et == "FDONE": self._fdone()
        elif et == "REP":   self._repair(a)
        elif et == "BATCH": self._batch(a)

    def _start(self, mid):
        m, p, gp = self.machines[mid], self.mp[mid], self.gp
        if m["state"] == "broken": return
        if random.random() < p["br"] / 100:
            m["state"] = "broken"; m["ss"] = self.now
            rt = clamp(rand_exp(p["rt"]) + 5, 5, 200)
            heapq.heappush(self.pq, (self.now + rt, "REP", mid, None, None))
            self._log("breakdown", mid, f"{p['name']} breakdown — repair {rt:.0f} min")
            return
        eff = clamp(rand_norm(1.0, gp["sigma"]), 0.4, 1.6)
        ct = p["ct"] / eff
        m["state"] = "running"; m["ss"] = self.now
        pid = self.pid; self.pid += 1
        heapq.heappush(self.pq, (self.now + ct, "DONE", mid, pid, None))
        self.energy += p["ew"] * (ct / 60)
        m["tool_wear"] = clamp(m["tool_wear"] + (ct / 60) * 0.015, 0, 1)

    def _done(self, mid, pid):
        m, gp = self.machines[mid], self.gp
        m["parts_run"] += 1; m["run_time"] += self.now - m["ss"]; m["state"] = "idle"
        self.transit.add(pid)
        tt = gp["travel"] + rand_exp(max(gp["tvar"], 0.5))
        heapq.heappush(self.pq, (self.now + tt, "ARV", None, pid, m["line"]))
        heapq.heappush(self.pq, (self.now + 0.2, "START", mid, None, None))

    def _arrive(self, pid, line):
        self.transit.discard(pid); gp = self.gp
        merge = {"A": gp["mA"], "B": gp["mB"], "C": gp["mC"]}[line]
        if line == "A": self.qA += 1
        elif line == "B": self.qB += 1
        else: self.qC += 1
        q = {"A": self.qA, "B": self.qB, "C": self.qC}[line]
        if not self.busy[line] and q >= merge:
            if line == "A": self.qA -= merge
            elif line == "B": self.qB -= merge
            else: self.qC -= merge
            self.busy[line] = True
            at = clamp(rand_norm(18, 4), 6, 45)
            heapq.heappush(self.pq, (self.now + at, "ADONE", line, None, None))
            self._log("assy_start", None, f"Assembly {line} started (batch {merge})")

    def _adone(self, line):
        self.busy[line] = False; self.line_out[line] += 1; gp = self.gp
        merge = {"A": gp["mA"], "B": gp["mB"], "C": gp["mC"]}[line]
        pid = self.pid; self.pid += 1; self.transit.add(pid)
        tt = gp["travel"] + rand_exp(max(gp["tvar"], 0.5))
        heapq.heappush(self.pq, (self.now + tt, "ARVF", None, None, None))
        q = {"A": self.qA, "B": self.qB, "C": self.qC}[line]
        if q >= merge:
            if line == "A": self.qA -= merge
            elif line == "B": self.qB -= merge
            else: self.qC -= merge
            self.busy[line] = True
            heapq.heappush(self.pq, (self.now + clamp(rand_norm(18, 4), 6, 45), "ADONE", line, None, None))
        self._log("assy_done", None, f"✓ Sub-assy {line} done (total {self.line_out[line]})")

    def _arvF(self):
        self.qF += 1; gp = self.gp
        if not self.busy["F"] and self.qF >= gp["mF"]:
            self.qF -= gp["mF"]; self.busy["F"] = True
            ft = clamp(rand_norm(28, 6), 10, 65)
            heapq.heappush(self.pq, (self.now + ft, "FDONE", None, None, None))

    def _fdone(self):
        self.busy["F"] = False; self.final_out += 1; gp = self.gp
        if self.qF >= gp["mF"]:
            self.qF -= gp["mF"]; self.busy["F"] = True
            heapq.heappush(self.pq, (self.now + clamp(rand_norm(28, 6), 10, 65), "FDONE", None, None, None))
        self._log("final", None, f"🏁 FINAL PRODUCT #{self.final_out} complete!")

    def _repair(self, mid):
        m = self.machines[mid]
        m["broken_time"] += self.now - m["ss"]; m["state"] = "idle"; m["ss"] = self.now
        heapq.heappush(self.pq, (self.now + 0.3, "START", mid, None, None))
        self._log("repair", mid, f"{self.mp[mid]['name']} repaired")

    def _batch(self, n):
        flushed = self.qA + self.qB + self.qC
        self.qA = self.qB = self.qC = 0; self.batches += 1
        heapq.heappush(self.pq, (self.now + self.gp["batch"] * 24 * 60, "BATCH", n + 1, None, None))
        self._log("batch", None, f"Batch #{n} cleanup — flushed {flushed} WIP at {self.now/60:.0f}h")

    def _log(self, etype, mid, msg):
        self.event_log.append({
            "time_h": round(self.now / 60, 2), "type": etype,
            "machine": self.mp[mid]["name"] if mid is not None else "—", "message": msg,
        })
        if len(self.event_log) > 2000: self.event_log = self.event_log[-1000:]

    def _snap(self):
        T = max(self.now, 1)
        for m in self.machines: m["util"] = m["run_time"] / T
        tb = sum(m["broken_time"] for m in self.machines)
        avail = max((T * 12 - tb) / (T * 12), 0)
        gp = self.gp
        oee = avail * 0.91 * gp["prim"] * gp["sec"] * 100
        h = self.now / 60
        self.ts["t"].append(round(h, 2)); self.ts["final"].append(self.final_out)
        self.ts["wip"].append(self.qA+self.qB+self.qC+self.qF+len(self.transit))
        self.ts["oee"].append(round(oee, 1)); self.ts["en"].append(round(self.energy, 1))
        self.ts["qA"].append(self.qA); self.ts["qB"].append(self.qB)
        self.ts["qC"].append(self.qC); self.ts["qF"].append(self.qF)

    def get_kpis(self):
        T = max(self.now, 1)
        for m in self.machines: m["util"] = m["run_time"] / T
        sorted_m = sorted(self.machines, key=lambda x: x["util"], reverse=True)
        sim_h = T / 60
        tb = sum(m["broken_time"] for m in self.machines)
        avail = max((T * 12 - tb) / (T * 12), 0)
        gp = self.gp
        oee = (avail * 0.91 * gp["prim"] * gp["sec"] * 100)
        # Shift info
        total_min = int(self.now)
        day = total_min // (24*60) + 1
        hh  = (total_min % (24*60)) // 60
        shift = "Morning 🌅" if 6<=hh<14 else "Afternoon 🌞" if 14<=hh<22 else "Night 🌙"
        return {
            "final_out": self.final_out,
            "line_out": dict(self.line_out),
            "oee": round(oee, 1),
            "sim_h": round(sim_h, 1),
            "tph": round(self.final_out / sim_h, 3) if sim_h > 0 else 0,
            "energy": round(self.energy, 0),
            "wip": self.qA+self.qB+self.qC+self.qF+len(self.transit),
            "qA": self.qA, "qB": self.qB, "qC": self.qC, "qF": self.qF,
            "batches": self.batches,
            "day": day, "shift": shift, "hour": hh,
            "machines": [{
                **m,
                "util_pct": round(m["util"]*100, 1),
                "wear_pct": round(m["tool_wear"]*100, 1),
                "broken_h": round(m["broken_time"]/60, 1),
            } for m in self.machines],
            "bottleneck": [m["id"] for m in sorted_m],
        }

# ─────────────────────────────────────────────
# SIDEBAR
# ─────────────────────────────────────────────
with st.sidebar:
    st.markdown("## 🏭 CNC Digital Twin")
    st.caption("Discrete Event Simulation · 3 lines · 12 machines · Stochastic model")
    st.markdown("---")

    with st.expander("⚙️ Simulation settings", expanded=True):
        sim_days = st.slider("Simulate (days)", 1, 90, 14)
        n_runs   = st.slider("Monte Carlo runs", 1, 15, 1, help="Multiple runs → confidence bands")

    run_btn = st.button("▶  Run simulation", type="primary", use_container_width=True)
    st.markdown("---")

    with st.expander("🔀 Assembly gates"):
        mA = st.slider("Line A merge", 1, 8, 4)
        mB = st.slider("Line B merge", 1, 8, 4)
        mC = st.slider("Line C merge", 1, 8, 4)
        mF = st.slider("Final merge",  1, 6, 3)

    with st.expander("🚚 Logistics"):
        travel = st.slider("Transfer time (min)", 1, 30, 5)
        tvar   = st.slider("Transfer variance",   0, 15, 3)

    with st.expander("📅 Scheduling"):
        batch_days = st.slider("Batch cleanup (days)", 3, 30, 7)

    with st.expander("📊 Stochastics"):
        sigma = st.slider("Efficiency σ", 0.01, 0.35, 0.12, 0.01,
                          help="Normal dist std dev for machine cycle time. Higher = more variability.")
        prim  = st.slider("Primary recovery",   0.80, 0.99, 0.95, 0.01,
                          help="Quality factor 1 — OEE Quality component")
        sec   = st.slider("Secondary recovery", 0.70, 0.95, 0.85, 0.01,
                          help="Quality factor 2 — OEE Quality component")

    st.markdown("---")
    st.markdown("### 🔧 Per-machine parameters")

    machine_params = []
    for d in BASE_DEFS:
        color = LINE_COLORS[d["line"]]
        with st.expander(f"{d['name']} · {d['desc']}"):
            ct2 = st.slider("Cycle time (min)", 2,  45,  d["ct"],  key=f"ct_{d['id']}")
            br2 = st.slider("Breakdown rate %", 0,  35,  d["br"],  key=f"br_{d['id']}")
            rt2 = st.slider("Repair time (min)", 5, 120, d["rt"],  key=f"rt_{d['id']}")
            ew2 = st.slider("Energy draw (kW)", 1,  25,  d["ew"],  key=f"ew_{d['id']}")
            machine_params.append(dict(id=d["id"], name=d["name"], line=d["line"],
                                       ct=ct2, br=br2, rt=rt2, ew=ew2, desc=d["desc"]))

    GLOBAL_PARAMS = dict(mA=mA, mB=mB, mC=mC, mF=mF, travel=travel, tvar=tvar,
                         batch=batch_days, sigma=sigma, prim=prim, sec=sec)

    st.markdown("---")
    st.markdown("""
**How it works**

This app runs a **Discrete Event Simulation** (DES) engine built from scratch — no SimPy.

- Priority queue (min-heap) drives Next-Event scheduling
- Normal distribution → cycle time variability
- Exponential distribution → repair times
- Bernoulli → breakdown decisions

[View source on GitHub ↗](https://github.com)
""")

# ─────────────────────────────────────────────
# HEADER
# ─────────────────────────────────────────────
st.title("🏭 CNC Production Line — Digital Twin")
st.caption("3 parallel CNC lines (A/B/C) · 4 machines each · 2-stage assembly · Stochastic DES · Monte Carlo support")

if not run_btn and "result" not in st.session_state:
    col1, col2, col3 = st.columns(3)
    with col1:
        st.info("**Configure** parameters in the sidebar")
    with col2:
        st.info("**Press ▶ Run** simulation to start")
    with col3:
        st.info("**Explore** results across 5 tabs")

    st.markdown("### How this works")
    c1, c2, c3, c4 = st.columns(4)
    c1.metric("DES Engine", "Custom PQ", "No SimPy needed")
    c2.metric("Machines", "12 CNC", "3 lines × 4 machines")
    c3.metric("Assembly", "3 + Final", "Parallel merge logic")
    c4.metric("Monte Carlo", "Up to 15×", "Confidence intervals")
    st.stop()

# ─────────────────────────────────────────────
# RUN
# ─────────────────────────────────────────────
if run_btn:
    sim_min = sim_days * 24 * 60
    all_kpis, all_logs, runs_ts = [], [], []

    prog = st.progress(0, text="Initialising simulation engine…")
    for ri in range(n_runs):
        prog.progress((ri) / n_runs, text=f"Run {ri+1} / {n_runs}  —  simulating {sim_days} days…")
        eng = DESEngine(machine_params, GLOBAL_PARAMS)
        eng.run(sim_min, snap_count=120)
        all_kpis.append(eng.get_kpis())
        all_logs.extend(eng.event_log[-300:])
        runs_ts.append(eng.ts)
    prog.empty()

    def avg_series(key):
        min_l = min(len(r[key]) for r in runs_ts)
        return [sum(r[key][i] for r in runs_ts) / n_runs for i in range(min_l)]

    ts_t      = runs_ts[0]["t"][:min(len(r["t"]) for r in runs_ts)]
    ts_final  = avg_series("final")
    ts_wip    = avg_series("wip")
    ts_oee    = avg_series("oee")
    ts_en     = avg_series("en")
    ts_qA, ts_qB, ts_qC, ts_qF = avg_series("qA"), avg_series("qB"), avg_series("qC"), avg_series("qF")

    kpi = all_kpis[-1]

    # Machine summary df
    rows = []
    for i, d in enumerate(BASE_DEFS):
        m = kpi["machines"][i]
        rows.append({
            "Machine": m["name"], "Line": m["line"], "Desc": d["desc"],
            "Util %": m["util_pct"], "Parts run": m["parts_run"],
            "Broken h": m["broken_h"], "Tool wear %": m["wear_pct"],
            "Cycle (min)": machine_params[i]["ct"], "Break %": machine_params[i]["br"],
        })
    df_mach = pd.DataFrame(rows)
    df_log  = pd.DataFrame(all_logs)

    st.session_state["result"] = dict(
        kpi=kpi, ts_t=ts_t, ts_final=ts_final, ts_wip=ts_wip, ts_oee=ts_oee,
        ts_en=ts_en, ts_qA=ts_qA, ts_qB=ts_qB, ts_qC=ts_qC, ts_qF=ts_qF,
        df_mach=df_mach, df_log=df_log, runs_ts=runs_ts, n_runs=n_runs,
        all_kpis=all_kpis,
    )
    st.rerun()

# ─────────────────────────────────────────────
# DISPLAY
# ─────────────────────────────────────────────
R   = st.session_state["result"]
kpi = R["kpi"]
ts_t = R["ts_t"]
df_mach = R["df_mach"]

PLC = dict(displayModeBar=False, responsive=True)
def PLL(**kw):
    base = dict(margin=dict(l=44,r=16,t=28,b=40), height=260,
                plot_bgcolor="#fff", paper_bgcolor="#fff",
                font=dict(size=11, family="system-ui"),
                xaxis=dict(showgrid=True, gridcolor="#f0f0f0", zeroline=False),
                yaxis=dict(showgrid=True, gridcolor="#f0f0f0", zeroline=False),
                showlegend=False)
    base.update(kw)
    return base

# ── TIME / SHIFT BANNER ──────────────────────
sh_col = {"Morning 🌅": "#E1F5EE", "Afternoon 🌞": "#FAEEDA", "Night 🌙": "#EEEDFE"}
sh_tc  = {"Morning 🌅": "#085041", "Afternoon 🌞": "#633806", "Night 🌙": "#3C3489"}
bg = sh_col.get(kpi["shift"], "#E6F1FB")
tc = sh_tc.get(kpi["shift"], "#0C447C")
st.markdown(f"""
<div style="background:{bg};border-radius:10px;padding:10px 18px;margin-bottom:12px;display:flex;align-items:center;gap:24px">
  <div>
    <span style="font-size:22px;font-weight:700;color:{tc}">Day {kpi['day']}</span>
    <span style="font-size:14px;color:{tc};margin-left:12px">{kpi['shift']}</span>
  </div>
  <div style="font-size:13px;color:{tc}">Sim time: <b>{kpi['sim_h']}h</b></div>
  <div style="font-size:13px;color:{tc}">Batches done: <b>{kpi['batches']}</b></div>
  <div style="font-size:13px;color:{tc}">In transit: <b>{len([])}</b></div>
</div>
""", unsafe_allow_html=True)

# ── KPI METRICS ──────────────────────────────
c1,c2,c3,c4,c5,c6,c7,c8 = st.columns(8)
oee_delta = f"{'✅' if kpi['oee']>=75 else '⚠️' if kpi['oee']>=55 else '❌'}"
c1.metric("Final output",    kpi["final_out"],          "units")
c2.metric("OEE",             f"{kpi['oee']}%",          oee_delta)
c3.metric("Throughput/h",    kpi["tph"],                "units/h")
c4.metric("Line A",          kpi["line_out"]["A"],       "sub-assy")
c5.metric("Line B",          kpi["line_out"]["B"],       "sub-assy")
c6.metric("Line C",          kpi["line_out"]["C"],       "sub-assy")
c7.metric("WIP total",       kpi["wip"],                 "parts")
c8.metric("Energy",          f"{int(kpi['energy']):,}",  "kWh")

st.markdown("---")

# ── TABS ─────────────────────────────────────
tab1, tab2, tab3, tab4, tab5 = st.tabs([
    "📊 Overview", "🔴 Bottleneck", "📈 Time series", "🏭 Machines", "📋 Event log"
])

# ── TAB 1: OVERVIEW ──────────────────────────
with tab1:
    col_l, col_r = st.columns([3, 2])
    with col_l:
        # Output trend + MC band
        fig_out = go.Figure()
        if R["n_runs"] > 1:
            min_l = min(len(r["final"]) for r in R["runs_ts"])
            arr   = [r["final"][:min_l] for r in R["runs_ts"]]
            p5  = [sorted(v[i] for v in arr)[int(len(arr)*0.05)] for i in range(min_l)]
            p95 = [sorted(v[i] for v in arr)[int(len(arr)*0.95)] for i in range(min_l)]
            fig_out.add_trace(go.Scatter(x=ts_t,y=p95,mode="lines",line=dict(color="rgba(37,99,235,0)"),showlegend=False))
            fig_out.add_trace(go.Scatter(x=ts_t,y=p5, mode="lines",fill="tonexty",fillcolor="rgba(37,99,235,0.10)",line=dict(color="rgba(37,99,235,0)"),name="5–95th pct"))
        fig_out.add_trace(go.Scatter(x=ts_t,y=R["ts_final"],mode="lines",line=dict(color="#16a34a",width=2.5),name="Avg output"))
        fig_out.update_layout(title="Final output over time", **PLL(showlegend=R["n_runs"]>1))
        st.plotly_chart(fig_out, use_container_width=True, config=PLC)

        fig_q = go.Figure()
        fig_q.add_trace(go.Scatter(x=ts_t,y=R["ts_qA"],name="Queue A",line=dict(color="#2563EB",width=1.8)))
        fig_q.add_trace(go.Scatter(x=ts_t,y=R["ts_qB"],name="Queue B",line=dict(color="#C2410C",width=1.8)))
        fig_q.add_trace(go.Scatter(x=ts_t,y=R["ts_qC"],name="Queue C",line=dict(color="#7C3AED",width=1.8)))
        fig_q.add_trace(go.Scatter(x=ts_t,y=R["ts_qF"],name="Final Q", line=dict(color="#16a34a",width=2,dash="dot")))
        fig_q.update_layout(title="Queue depths — all stages",showlegend=True,**PLL(legend=dict(orientation="h",y=-0.35)))
        st.plotly_chart(fig_q, use_container_width=True, config=PLC)

    with col_r:
        fig_oee = go.Figure()
        fig_oee.add_trace(go.Scatter(x=ts_t,y=R["ts_oee"],mode="lines",fill="tozeroy",line=dict(color="#0F6E56",width=2),fillcolor="rgba(15,110,86,0.08)"))
        fig_oee.add_hline(y=75,line_dash="dash",line_color="#dc2626",annotation_text="75% target")
        fig_oee.update_layout(title="OEE % over time",**PLL(yaxis=dict(range=[0,100])))
        st.plotly_chart(fig_oee, use_container_width=True, config=PLC)

        fig_en = go.Figure()
        fig_en.add_trace(go.Scatter(x=ts_t,y=R["ts_en"],mode="lines",fill="tozeroy",line=dict(color="#d97706",width=2),fillcolor="rgba(217,119,6,0.08)"))
        fig_en.update_layout(title="Cumulative energy (kWh)",**PLL())
        st.plotly_chart(fig_en, use_container_width=True, config=PLC)

        lo = kpi["line_out"]
        fig_pie = go.Figure(go.Pie(
            labels=["Line A","Line B","Line C"],values=[lo["A"],lo["B"],lo["C"]],
            hole=0.42,marker_colors=["#2563EB","#C2410C","#7C3AED"],textinfo="label+percent"))
        fig_pie.update_layout(title="Line balance",margin=dict(l=10,r=10,t=40,b=10),height=220,paper_bgcolor="#fff")
        st.plotly_chart(fig_pie, use_container_width=True, config=PLC)

# ── TAB 2: BOTTLENECK ────────────────────────
with tab2:
    ms = kpi["machines"]
    utils  = [m["util_pct"] for m in ms]
    names  = [m["name"] for m in ms]
    ucols  = ["#dc2626" if u>80 else "#f59e0b" if u>65 else "#22c55e" for u in utils]

    fig_bn = go.Figure()
    fig_bn.add_trace(go.Bar(x=names,y=utils,marker_color=ucols,text=[f"{u}%" for u in utils],textposition="outside"))
    fig_bn.add_hline(y=80,line_dash="dash",line_color="#dc2626",annotation_text="Critical (80%)")
    fig_bn.add_hline(y=65,line_dash="dot", line_color="#f59e0b",annotation_text="Warning (65%)")
    fig_bn.update_layout(title="Machine utilisation — bottleneck ranking",**PLL(height=320,yaxis=dict(range=[0,115])))
    st.plotly_chart(fig_bn, use_container_width=True, config=PLC)

    c1, c2 = st.columns(2)
    with c1:
        wears = [m["wear_pct"] for m in ms]
        wcols = ["#dc2626" if w>70 else "#f59e0b" if w>40 else "#22c55e" for w in wears]
        fig_w = go.Figure(go.Bar(x=names,y=wears,marker_color=wcols,text=[f"{w}%" for w in wears],textposition="outside"))
        fig_w.add_hline(y=70,line_dash="dash",line_color="#dc2626",annotation_text="Replace threshold")
        fig_w.update_layout(title="Tool wear %",**PLL(height=280,yaxis=dict(range=[0,120])))
        st.plotly_chart(fig_w, use_container_width=True, config=PLC)

    with c2:
        run_h   = [m["parts_run"] * machine_params[i]["ct"] / 60 for i,m in enumerate(ms)]
        broken_h= [m["broken_h"] for m in ms]
        fig_bt = go.Figure()
        fig_bt.add_trace(go.Bar(name="Run time",    x=names,y=run_h,    marker_color="#2563EB"))
        fig_bt.add_trace(go.Bar(name="Broken time", x=names,y=broken_h, marker_color="#dc2626"))
        fig_bt.update_layout(title="Run vs broken time (h)",barmode="stack",showlegend=True,
                             legend=dict(orientation="h",y=-0.35),**PLL(height=280))
        st.plotly_chart(fig_bt, use_container_width=True, config=PLC)

    st.markdown("#### Bottleneck table")
    df_disp = df_mach.sort_values("Util %", ascending=False)
    def color_rows(row):
        u = row["Util %"]
        c = "background-color:#fff1f2" if u>80 else "background-color:#fffbeb" if u>65 else ""
        return [c]*len(row)
    st.dataframe(
        df_disp.style.apply(color_rows,axis=1).format({"Util %":"{:.1f}%","Tool wear %":"{:.1f}%","Broken h":"{:.1f}h"}),
        use_container_width=True, height=380,
    )

# ── TAB 3: TIME SERIES ───────────────────────
with tab3:
    fig4 = make_subplots(rows=2,cols=2,subplot_titles=("Final output","WIP total","OEE %","Cumulative energy"),
                         vertical_spacing=0.18,horizontal_spacing=0.10)
    fig4.add_trace(go.Scatter(x=ts_t,y=R["ts_final"],mode="lines",line=dict(color="#16a34a",width=2)),row=1,col=1)
    fig4.add_trace(go.Scatter(x=ts_t,y=R["ts_wip"],  mode="lines",line=dict(color="#d97706",width=2)),row=1,col=2)
    fig4.add_trace(go.Scatter(x=ts_t,y=R["ts_oee"],  mode="lines",line=dict(color="#0F6E56",width=2)),row=2,col=1)
    fig4.add_trace(go.Scatter(x=ts_t,y=R["ts_en"],   mode="lines",line=dict(color="#C2410C",width=2)),row=2,col=2)
    for r in [1,2]:
        for c in [1,2]:
            fig4.update_xaxes(showgrid=True,gridcolor="#f0f0f0",row=r,col=c)
            fig4.update_yaxes(showgrid=True,gridcolor="#f0f0f0",row=r,col=c)
    fig4.update_layout(height=520,showlegend=False,plot_bgcolor="#fff",paper_bgcolor="#fff",
                       margin=dict(l=50,r=20,t=50,b=40),font=dict(size=11))
    st.plotly_chart(fig4, use_container_width=True, config=PLC)

# ── TAB 4: MACHINES ──────────────────────────
with tab4:
    st.dataframe(
        df_mach.style
        .background_gradient(subset=["Util %"],    cmap="RdYlGn_r", vmin=0, vmax=100)
        .background_gradient(subset=["Tool wear %"],cmap="RdYlGn_r", vmin=0, vmax=100)
        .background_gradient(subset=["Broken h"],  cmap="Reds",     vmin=0)
        .format({"Util %":"{:.1f}%","Tool wear %":"{:.1f}%","Broken h":"{:.1f}h"}),
        use_container_width=True, height=400,
    )
    c1, c2 = st.columns(2)
    with c1:
        fig_p = px.bar(df_mach.sort_values("Parts run"),x="Parts run",y="Machine",
                       color="Line",color_discrete_map=LINE_COLORS,orientation="h",
                       title="Parts produced per machine",height=360)
        fig_p.update_layout(plot_bgcolor="#fff",paper_bgcolor="#fff",margin=dict(l=20,r=20,t=40,b=30))
        st.plotly_chart(fig_p, use_container_width=True, config=PLC)
    with c2:
        # Heatmap
        z   = [[kpi["machines"][i]["util_pct"] for i in range(j*4, j*4+4)] for j in range(3)]
        fig_h = go.Figure(go.Heatmap(z=z,x=["M1","M2","M3","M4"],y=["Line A","Line B","Line C"],
            colorscale="RdYlGn",reversescale=True,zmin=0,zmax=100,
            text=[[f"{v:.0f}%" for v in row] for row in z],texttemplate="%{text}",
            colorbar=dict(title="Util %")))
        fig_h.update_layout(title="Utilisation heatmap",height=360,
                            margin=dict(l=60,r=60,t=40,b=40),paper_bgcolor="#fff")
        st.plotly_chart(fig_h, use_container_width=True, config=PLC)

# ── TAB 5: EVENT LOG ─────────────────────────
with tab5:
    if not R["df_log"].empty:
        type_filter = st.multiselect(
            "Filter event types",
            options=R["df_log"]["type"].unique().tolist(),
            default=["final","breakdown","batch","assy_done","repair"],
        )
        df_f = R["df_log"][R["df_log"]["type"].isin(type_filter)] if type_filter else R["df_log"]

        def color_ev(row):
            c = {"final":"background-color:#f0fdf4","breakdown":"background-color:#fff1f2",
                 "repair":"background-color:#f0fdf4","batch":"background-color:#fffbeb",
                 "assy_done":"background-color:#eff6ff"}
            return [c.get(row["type"],"")] * len(row)

        st.dataframe(
            df_f.sort_values("time_h",ascending=False).reset_index(drop=True)
            .style.apply(color_ev,axis=1).format({"time_h":"{:.2f}h"}),
            use_container_width=True, height=420,
        )
        fig_ev = px.bar(R["df_log"]["type"].value_counts().reset_index().rename(columns={"index":"type","type":"count","count":"count"}),
                        x="type",y="count",title="Event frequency",height=250,
                        color_discrete_sequence=px.colors.qualitative.Set2)
        fig_ev.update_layout(plot_bgcolor="#fff",paper_bgcolor="#fff",showlegend=False,
                             margin=dict(l=40,r=20,t=40,b=40))
        st.plotly_chart(fig_ev, use_container_width=True, config=PLC)
