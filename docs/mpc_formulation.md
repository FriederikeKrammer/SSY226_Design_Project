# MPC formulation (draft for team discussion)

Status: **draft, v0.1** — written as a starting point for week 1. Everything marked
**[decide]** is an open modelling choice for the team; suggested defaults are given.

## 1. Overview

At every hour $t$ of the simulated year the controller:

1. reads the current state (BESS energy, EV energy),
2. takes forecasts for the next $N$ hours (load, PV, EV availability, driving energy),
3. solves the optimisation problem below over the horizon $k = 0, \dots, N-1$,
4. applies **only the first step** ($k=0$) to the simulated system,
5. advances one hour and repeats.

This is the *receding-horizon* principle: the plan is recomputed every hour, so forecast
errors and new information are taken into account. A full year = 8,760 solves.

- Time step $\Delta t = 1$ h. All flows are **energy per step in kWh**, which numerically
  equals average power in kW over that hour — so no $\Delta t$ factors appear below.
- Horizon $N$ **[decide]**: suggested 24–48 h (one to two days of solar cycle).

## 2. Energy flows (decision variables)

Allowed flows follow Fig. 1 of the project plan. All variables are $\ge 0$.

| Variable | From → To | Fig. 1 | Notes |
|---|---|---|---|
| $P^{\mathrm{pv,h}}_k$ | PV → Home | solid | |
| $P^{\mathrm{pv,b}}_k$ | PV → BESS | solid | |
| $P^{\mathrm{pv,ev}}_k$ | PV → EV | solid | |
| $P^{\mathrm{cur}}_k$ | PV → curtailed | — | needed: Fig. 1 has no PV → Grid export, so surplus must be spilled |
| $P^{\mathrm{b,h}}_k$ | BESS → Home | solid | |
| $P^{\mathrm{b,ev}}_k$ | BESS → EV | solid | |
| $P^{\mathrm{ev,h}}_k$ | EV → Home (V2H) | solid | zero in the unidirectional baseline |
| $P^{\mathrm{ev,g}}_k$ | EV → Grid (V2G) | dashed | **[decide]** suggest fixing to 0 (does not help autonomy) |
| $P^{\mathrm{g,h}}_k$ | Grid → Home | dashed | grid import |
| $P^{\mathrm{g,b}}_k$ | Grid → BESS | dashed | grid import |
| $P^{\mathrm{g,ev}}_k$ | Grid → EV | dashed | grid import |

Not allowed (absent from Fig. 1): PV → Grid, EV → BESS, BESS → Grid.

**States:** $E^{\mathrm{b}}_k$ (BESS stored energy, kWh) and $E^{\mathrm{ev}}_k$ (EV stored
energy, kWh) for $k = 0, \dots, N$. $E^{\mathrm{b}}_0$ and $E^{\mathrm{ev}}_0$ are fixed to the
measured state at time $t$.

Short-hands used below:

$$
\begin{aligned}
C^{\mathrm{b}}_k &= P^{\mathrm{pv,b}}_k + P^{\mathrm{g,b}}_k
  &&\text{BESS charging} \\
D^{\mathrm{b}}_k &= P^{\mathrm{b,h}}_k + P^{\mathrm{b,ev}}_k
  &&\text{BESS discharging} \\
C^{\mathrm{ev}}_k &= P^{\mathrm{pv,ev}}_k + P^{\mathrm{b,ev}}_k + P^{\mathrm{g,ev}}_k
  &&\text{EV charging} \\
D^{\mathrm{ev}}_k &= P^{\mathrm{ev,h}}_k + P^{\mathrm{ev,g}}_k
  &&\text{EV discharging} \\
G_k &= P^{\mathrm{g,h}}_k + P^{\mathrm{g,b}}_k + P^{\mathrm{g,ev}}_k
  &&\text{total grid import}
\end{aligned}
$$

## 3. Inputs (forecasts over the horizon)

| Symbol | Meaning | Source |
|---|---|---|
| $L_k$ | household load, kWh | `cons` column |
| $PV_k$ | available PV energy, kWh | `pv.pv_generation(...)` |
| $a_k \in \{0,1\}$ | EV at home and plugged in | EV schedule **[decide]** |
| $d_k$ | EV energy used for driving in step $k$, kWh | EV schedule **[decide]** |

Baseline forecast **[decide]**: *perfect foresight* (use the actual future data). This gives
an upper bound on what MPC can achieve; forecast errors (e.g. persistence forecast) can be
added later as a sensitivity study.

## 4. Constraints

For all $k = 0, \dots, N-1$:

**PV split** — available PV is used or curtailed:

$$P^{\mathrm{pv,h}}_k + P^{\mathrm{pv,b}}_k + P^{\mathrm{pv,ev}}_k + P^{\mathrm{cur}}_k = PV_k$$

**Household energy balance** — the load is always met:

$$P^{\mathrm{pv,h}}_k + P^{\mathrm{b,h}}_k + P^{\mathrm{ev,h}}_k + P^{\mathrm{g,h}}_k = L_k$$

**BESS dynamics** (charging efficiency $\eta^{\mathrm{b}}_c$, discharging efficiency $\eta^{\mathrm{b}}_d$):

$$E^{\mathrm{b}}_{k+1} = E^{\mathrm{b}}_k + \eta^{\mathrm{b}}_c C^{\mathrm{b}}_k - \frac{1}{\eta^{\mathrm{b}}_d} D^{\mathrm{b}}_k$$

**BESS limits** (capacity $\bar E^{\mathrm{b}}$, power rating $\bar P^{\mathrm{b}}$):

$$
\underline{s}^{\mathrm{b}} \bar E^{\mathrm{b}} \le E^{\mathrm{b}}_{k+1} \le \overline{s}^{\mathrm{b}} \bar E^{\mathrm{b}},
\qquad C^{\mathrm{b}}_k \le \bar P^{\mathrm{b}},
\qquad D^{\mathrm{b}}_k \le \bar P^{\mathrm{b}}
$$

**EV dynamics** — driving energy leaves the battery:

$$E^{\mathrm{ev}}_{k+1} = E^{\mathrm{ev}}_k + \eta^{\mathrm{ev}}_c C^{\mathrm{ev}}_k - \frac{1}{\eta^{\mathrm{ev}}_d} D^{\mathrm{ev}}_k - d_k$$

**EV limits** — charging/discharging only while plugged in ($a_k = 1$); $\beta = 0$ for the
unidirectional baseline, $\beta = 1$ for bidirectional (V2H):

$$
\underline{s}^{\mathrm{ev}} \bar E^{\mathrm{ev}} \le E^{\mathrm{ev}}_{k+1} \le \overline{s}^{\mathrm{ev}} \bar E^{\mathrm{ev}},
\qquad C^{\mathrm{ev}}_k \le a_k \bar P^{\mathrm{ev}},
\qquad D^{\mathrm{ev}}_k \le \beta\, a_k \bar P^{\mathrm{ev}}
$$

**Departure requirement** **[decide]** — the car must be charged enough to drive. At each
departure step $k \in \mathcal{K}_{\mathrm{dep}}$:

$$E^{\mathrm{ev}}_k \ge E^{\mathrm{dep}}$$

Without this, V2H could empty the car before a trip. The minimum SoC
$\underline{s}^{\mathrm{ev}}$ alone also protects the battery, but $E^{\mathrm{dep}}$ expresses
the user's mobility need (e.g. 80 % before leaving).

## 5. Objective

Minimise grid import (the plan's autonomy goal), plus small secondary terms:

$$
\min \sum_{k=0}^{N-1} \Big[\, G_k
  \;+\; w_{\mathrm{deg}}^{\mathrm{b}} \big(C^{\mathrm{b}}_k + D^{\mathrm{b}}_k\big)
  \;+\; w_{\mathrm{deg}}^{\mathrm{ev}} \big(C^{\mathrm{ev}}_k + D^{\mathrm{ev}}_k\big)
  \;+\; w_{\mathrm{v2g}} P^{\mathrm{ev,g}}_k \Big]
  \;-\; w_{\mathrm{term}} \big(E^{\mathrm{b}}_N + E^{\mathrm{ev}}_N\big)
$$

- **Grid import $G_k$** has weight 1 and dominates. Note Grid→BESS and Grid→EV count as
  import too — there is no reason to charge from the grid except to meet a driving need.
- **Degradation** (simplified, per plan): a throughput penalty — every kWh in or out of a
  battery costs a little. Keeps batteries from cycling needlessly. Weights small, e.g.
  $10^{-3}$–$10^{-2}$, so they never outweigh avoiding 1 kWh of import. **[decide]** whether
  to add an SoC-dependent ageing term later (calendar ageing at high SoC).
- **Terminal value** $w_{\mathrm{term}}$: without it the optimiser sees stored energy at the
  end of the horizon as worthless and empties the batteries "for free". A value slightly
  below 1 (e.g. $0.9\,\eta_d$) makes it keep energy that will be useful after the horizon.
  This matters a lot with a short horizon. Alternative: a terminal constraint
  $E^{\mathrm{b}}_N \ge E^{\mathrm{b}}_0$.

**Why this is an LP:** all constraints and the objective are linear in the variables, and all
variables are continuous. HiGHS solves an LP of this size ($\approx 13N$ variables, e.g. ~600
for $N=48$) in milliseconds.

**Simultaneous charge/discharge:** an LP could charge and discharge a battery in the same
hour. With losses ($\eta < 1$) plus the degradation penalty this is never optimal, *except*
as a way to burn surplus PV — and since curtailment is free, the solver prefers curtailing.
So no binary variables are needed; if it shows up in results, add binaries (→ MILP).

## 6. Parameters (suggested starting values **[decide]**)

| Parameter | Symbol | Suggested | Comment |
|---|---|---|---|
| PV size | $PV^{\mathrm{size}}$ | 5–20 kWp | sweep |
| BESS capacity | $\bar E^{\mathrm{b}}$ | 0–30 kWh | sweep |
| BESS power | $\bar P^{\mathrm{b}}$ | 0.5 C (= cap/2 per hour) | |
| BESS efficiencies | $\eta^{\mathrm{b}}_c, \eta^{\mathrm{b}}_d$ | 0.95, 0.95 | ~90 % round trip |
| BESS SoC window | $\underline{s}^{\mathrm{b}}, \overline{s}^{\mathrm{b}}$ | 0.1, 0.9 | |
| EV capacity | $\bar E^{\mathrm{ev}}$ | 40–80 kWh | sweep |
| Charger power | $\bar P^{\mathrm{ev}}$ | 11 kW | 3-phase home charger; also test 3.7 kW |
| EV efficiencies | $\eta^{\mathrm{ev}}_c, \eta^{\mathrm{ev}}_d$ | 0.92, 0.92 | |
| EV SoC window | $\underline{s}^{\mathrm{ev}}, \overline{s}^{\mathrm{ev}}$ | 0.2, 0.9 | |
| Departure energy | $E^{\mathrm{dep}}$ | 0.8 $\bar E^{\mathrm{ev}}$ | or "enough for next trip + margin" |
| Driving consumption | — | 0.18 kWh/km | |
| Horizon | $N$ | 24–48 h | |

## 7. EV schedule (inputs $a_k$, $d_k$) **[decide]**

Simple baseline, to be refined in week 3 ("realistic driving schedules"):

- Weekdays: away 07:00–17:00 ($a_k = 0$), 40 km round trip → 7.2 kWh, spread over the away
  hours (or subtracted at departure).
- Weekends: one shorter trip, or at home all day.
- Later: stochastic or survey-based schedules (e.g. Swedish travel survey data), longer
  trips, holidays. Availability is a key sensitivity parameter in the research question.

## 8. Simulation and metrics

The `simulation` module runs the receding-horizon loop and applies the first step to a
"plant" using the same equations (state update for $E^{\mathrm{b}}$, $E^{\mathrm{ev}}$).
`metrics` computes, over a year:

- **Grid import** $\sum_t G_t$ (kWh). Annual autonomy ⇔ $\sum_t G_t = 0$.
- **Self-sufficiency** $SS = 1 - \dfrac{\sum_t G_t}{\sum_t (L_t + d_t)}$ — **[decide]**
  whether EV driving energy counts as demand (Gstöhl & Pfenninger include it).
- **Curtailed PV**, battery throughput (proxy for degradation), hours with import.

For the handout's research question: for each (PV, EV, schedule) case, find the smallest
$\bar E^{\mathrm{b}}$ that reaches a target $SS$ with $\beta = 0$ and with $\beta = 1$. The
difference is the BESS capacity avoided by V2H.

## 9. Implementation notes (CVXPY)

- Build the problem **once** with `cp.Parameter`s for $L$, $PV$, $a$, $d$, $E^{\mathrm{b}}_0$,
  $E^{\mathrm{ev}}_0$; each hour update `param.value` and call `prob.solve(solver=cp.HIGHS)`.
  CVXPY then reuses the compiled problem → fast.
- Keep parameter expressions simple (CVXPY's DPP rules restrict how parameters may be
  combined): precompute e.g. $\beta\,a_k\bar P^{\mathrm{ev}}$ in numpy and pass it as one
  parameter `ev_dis_max`.
- Component models (`battery.py`, `ev.py`) should hold the parameters (e.g. a dataclass) and
  maybe helper functions; `mpc.py` assembles the constraints.

## 10. Open questions for the team

1. Horizon length $N$ — and is a short horizon enough, given the real challenge is
   *seasonal* (summer surplus, winter deficit)? A daily-cycle battery cannot bridge seasons;
   this is likely a key finding rather than a modelling flaw.
2. Allow V2G (EV → Grid)? Suggest no — export does not improve autonomy.
3. Should EV driving energy count as household demand in self-sufficiency?
4. Departure requirement: fixed $E^{\mathrm{dep}}$, or energy needed for the next trip?
5. What if the EV cannot meet a trip from PV/BESS? It must charge from the grid (counted as
   import) — or model "public charging" separately and exclude it from household import?
6. Perfect foresight vs. realistic forecasts as the baseline.
7. Degradation model: throughput penalty only, or something more detailed?
