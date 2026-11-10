import os

import numpy as np
import pandas as pd
import plotly.graph_objects as go
import plotly.io as pio
import pyomo.environ as pyo

# load data
df = pd.read_csv("data_HL-T-sun_2021-2025.csv")
df["datetime"] = pd.to_datetime(df["datetime"])
df = df.sort_values("datetime").reset_index(drop=True)
dt = df["datetime"].diff().dt.total_seconds().median() / 3600
demand = df["cons"].clip(lower=0).to_numpy() * dt


## System parameters
# PV system
PV_capacity = 5.0  # kWp
PV_efficiency = 0.90

# BESS
BESS_capacity = 10.0  # kWh
BESS_power = 5.0  # kW
BESS_eta_ch = 0.95
BESS_eta_dis = 0.95

BESS_SOC_min = 0.10 * BESS_capacity
BESS_SOC_max = 0.90 * BESS_capacity
BESS_SOC_initial = 0.50 * BESS_capacity

# EV
EV_capacity = 60.0  # kWh
EV_power = 7.4  # kW
EV_eta = 0.95

EV_SOC_min = 0.10 * EV_capacity
EV_SOC_max = 0.90 * EV_capacity
EV_SOC_initial = 0.60 * EV_capacity

# EV driving assumptions
EV_daily_driving = 5.0  # kWh per day

# Grid
Grid_power_max = 15.0  # kW

# MPC
H = 24  # Prediction horizon (24 hours)

# Objective weights
c_grid = 1.0
c_export = 0.02
c_deg = 0.01
c_curt = 0.001


# PV generation
irradiance = df["sun"].clip(lower=0).to_numpy()
PV_power = PV_capacity * irradiance * PV_efficiency
PV_power = np.minimum(PV_power, PV_capacity)
PV_energy = PV_power * dt


# EV availability/driving profile
hours = df["datetime"].dt.hour.to_numpy()
# EV connected at home between 18:00 and 07:00
EV_available = ((hours >= 18) | (hours < 7)).astype(float)
# Driving consumption at 07:00
EV_drive = np.where(hours == 7, EV_daily_driving, 0.0)


# MPC optimazation function
def solve_mpc(demand_h, pv_h, availability_h, drive_h, soc_bess, soc_ev):
    N = len(demand_h)
    m = pyo.ConcreteModel()
    m.T = pyo.RangeSet(0, N - 1)
    m.S = pyo.RangeSet(0, N)

    # Decision variables
    # Grid
    m.grid_import = pyo.Var(m.T, domain=pyo.NonNegativeReals)
    m.grid_export = pyo.Var(m.T, domain=pyo.NonNegativeReals)

    # PV
    m.pv_used = pyo.Var(m.T, domain=pyo.NonNegativeReals)
    m.pv_curt = pyo.Var(m.T, domain=pyo.NonNegativeReals)

    # BESS
    m.bess_ch = pyo.Var(m.T, domain=pyo.NonNegativeReals)
    m.bess_dis = pyo.Var(m.T, domain=pyo.NonNegativeReals)
    m.soc_bess = pyo.Var(m.S, bounds=(BESS_SOC_min, BESS_SOC_max))

    # EV (unidirectional charging)
    m.ev_ch = pyo.Var(m.T, domain=pyo.NonNegativeReals)
    m.soc_ev = pyo.Var(m.S, bounds=(EV_SOC_min, EV_SOC_max))

    # Binary variables
    m.bess_mode = pyo.Var(m.T, domain=pyo.Binary)
    m.grid_mode = pyo.Var(m.T, domain=pyo.Binary)

    # Initial SOC
    m.initial_bess = pyo.Constraint(expr=m.soc_bess[0] == soc_bess)
    m.initial_ev = pyo.Constraint(expr=m.soc_ev[0] == soc_ev)

    # Energy balance
    def energy_balance(m, t):
        return (
            m.pv_used[t] + m.grid_import[t] + m.bess_dis[t]
            == demand_h[t] + m.ev_ch[t] + m.bess_ch[t] + m.grid_export[t]
        )

    m.energy_balance = pyo.Constraint(m.T, rule=energy_balance)

    # PV constraints
    def pv_balance(m, t):
        return m.pv_used[t] + m.pv_curt[t] == pv_h[t]

    m.pv_balance = pyo.Constraint(m.T, rule=pv_balance)

    # BESS constraints
    def bess_soc(m, t):
        return (
            m.soc_bess[t + 1]
            == m.soc_bess[t] + BESS_eta_ch * m.bess_ch[t] - m.bess_dis[t] / BESS_eta_dis
        )

    m.bess_soc = pyo.Constraint(m.T, rule=bess_soc)

    # Charge/discharge power limits
    def bess_charge_limit(m, t):
        return m.bess_ch[t] <= BESS_power * dt * m.bess_mode[t]

    def bess_discharge_limit(m, t):
        return m.bess_dis[t] <= BESS_power * dt * (1 - m.bess_mode[t])

    m.bess_charge_limit = pyo.Constraint(m.T, rule=bess_charge_limit)
    m.bess_discharge_limit = pyo.Constraint(m.T, rule=bess_discharge_limit)

    # EV constraints
    def ev_soc(m, t):
        return m.soc_ev[t + 1] == m.soc_ev[t] + EV_eta * m.ev_ch[t] - drive_h[t]

    m.ev_soc = pyo.Constraint(m.T, rule=ev_soc)

    def ev_charge_limit(m, t):
        return m.ev_ch[t] <= EV_power * dt * availability_h[t]

    m.ev_charge_limit = pyo.Constraint(m.T, rule=ev_charge_limit)

    # Grid constraints
    def grid_import_limit(m, t):
        return m.grid_import[t] <= Grid_power_max * dt * m.grid_mode[t]

    def grid_export_limit(m, t):
        return m.grid_export[t] <= Grid_power_max * dt * (1 - m.grid_mode[t])

    m.grid_import_limit = pyo.Constraint(m.T, rule=grid_import_limit)
    m.grid_export_limit = pyo.Constraint(m.T, rule=grid_export_limit)

    # Terminal SOC constraints to avoid artificially emptying the BESS at horizon end
    m.terminal_bess = pyo.Constraint(expr=m.soc_bess[N] >= soc_bess)

    # OBJECTIVE FUNCTION
    def objective(m):
        return sum(
            c_grid * m.grid_import[t]
            + c_export * m.grid_export[t]
            + c_deg * (m.bess_ch[t] + m.bess_dis[t])
            + c_curt * m.pv_curt[t]
            for t in m.T
        )

    m.objective = pyo.Objective(rule=objective, sense=pyo.minimize)

    # Solve optimization problem
    solver = pyo.SolverFactory("appsi_highs")
    results = solver.solve(m, tee=False)
    if results.solver.termination_condition != (pyo.TerminationCondition.optimal):
        raise RuntimeError(
            f"MPC optimization failed: {results.solver.termination_condition}"
        )

    # Return first control action and next SOC
    return {
        "grid_import": pyo.value(m.grid_import[0]),
        "grid_export": pyo.value(m.grid_export[0]),
        "pv_used": pyo.value(m.pv_used[0]),
        "pv_curt": pyo.value(m.pv_curt[0]),
        "bess_ch": pyo.value(m.bess_ch[0]),
        "bess_dis": pyo.value(m.bess_dis[0]),
        "ev_ch": pyo.value(m.ev_ch[0]),
        "soc_bess": pyo.value(m.soc_bess[1]),
        "soc_ev": pyo.value(m.soc_ev[1]),
    }


# MPC Simulation
# Initial SOC
soc_bess = BESS_SOC_initial
soc_ev = EV_SOC_initial

# Simulation steps
simulation_steps = min(len(df), 7 * 24)
results = []

for k in range(simulation_steps):
    end = min(k + H, len(df))  # Prediction horizon
    demand_h = demand[k:end]
    pv_h = PV_energy[k:end]
    availability_h = EV_available[k:end]
    drive_h = EV_drive[k:end]
    solution = solve_mpc(demand_h, pv_h, availability_h, drive_h, soc_bess, soc_ev)

    # Implement ONLY the first control action
    soc_bess = solution["soc_bess"]
    soc_ev = solution["soc_ev"]
    results.append(solution)


# 7Results
res = pd.DataFrame(results)
res["datetime"] = df["datetime"].iloc[:simulation_steps].reset_index(drop=True)

# Energy autonomy KPI
total_demand = (demand[:simulation_steps] + EV_drive[:simulation_steps]).sum()
total_grid_import = res["grid_import"].sum()
energy_autonomy = (1 - total_grid_import / total_demand) * 100

print("--------------------------------")
print("MPC Simulation results")
print("--------------------------------")
print(f"Grid import: {total_grid_import:.2f} kWh")
print(f"Grid export: {res['grid_export'].sum():.2f} kWh")
print(f"Energy autonomy: {energy_autonomy:.2f} %")

# Sanity check results
print("Number of simulation results:", len(res))
print("Available columns:")
print(res.columns.tolist())
if res.empty:
    raise ValueError("No MPC results available!")

# Plot 1: Battery SOC
fig1 = go.Figure()
fig1.add_trace(
    go.Scatter(x=res["datetime"], y=res["soc_bess"], name="BESS SOC", mode="lines")
)
fig1.add_trace(
    go.Scatter(x=res["datetime"], y=res["soc_ev"], name="EV SOC", mode="lines")
)
fig1.update_layout(
    title="Battery State of Charge",
    xaxis_title="Time",
    yaxis_title="SOC [kWh]",
    template="plotly_white",
    hovermode="x unified",
    height=650,
)

# Plot 2: Grid import and export
fig2 = go.Figure()
fig2.add_trace(
    go.Scatter(
        x=res["datetime"], y=res["grid_import"], name="Grid Import", mode="lines"
    )
)
fig2.add_trace(
    go.Scatter(
        x=res["datetime"], y=res["grid_export"], name="Grid Export", mode="lines"
    )
)
fig2.update_layout(
    title="Grid Energy Exchange",
    xaxis_title="Time",
    yaxis_title="Energy [kWh]",
    template="plotly_white",
    hovermode="x unified",
    height=650,
)

# Plot 3: energy flows
fig3 = go.Figure()
for col in ["pv_used", "bess_ch", "bess_dis", "ev_ch"]:
    fig3.add_trace(go.Scatter(x=res["datetime"], y=res[col], name=col, mode="lines"))
fig3.update_layout(
    title="Energy Management and Power Flows",
    xaxis_title="Time",
    yaxis_title="Energy [kWh]",
    template="plotly_white",
    hovermode="x unified",
    height=650,
)

# Save plots
os.makedirs("MPC_plots", exist_ok=True)
pio.write_html(fig1, file="MPC_plots/Battery_SOC.html", auto_open=False)
pio.write_html(fig2, file="MPC_plots/Grid_Exchange.html", auto_open=False)
pio.write_html(fig3, file="MPC_plots/Energy_Flows.html", auto_open=False)
print("\nPlots successfully saved!")
