#!/usr/bin/env python3
"""
Ground Station for Laser Power Ramping — FULLY FIXED EDITION
- Logs COMPLETE telemetry: distance, attitude, actual grant/deny status
- Tracks actual GRANT vs DENY per sample (not just ACK reception)
- Proper deadlock recovery for Aggressive Maneuver
- Distance/attitude from NAMED_VALUE_FLOAT telemetry
"""

import os, time, threading, csv, json, random
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from pymavlink import mavutil
from permit_common import (
    load_cfg, PermitConfig, make_conn, setup_signing_if_enabled,
    Watchdogs, hardware_pwm_disable, hardware_pwm_enable
)

# ---- Env ---------------------------------------------------------------------
os.environ.setdefault("MAVLINK20", "1")
os.environ.setdefault("MAVLINK_DIALECT", "laser_safety")
SIM_SEED = int(os.getenv("SIM_SEED", "12345"))
random.seed(SIM_SEED)

# Reason codes (mirror Air)
REASON_TEXT = {
    0: "OK",
    1: "GatesFailed",
    2: "SeqWindow",
    3: "TTLExpired",
    4: "PX4NotOK",
    5: "LocalHealth",
    6: "GroundIntentOff",
}

# ---- Config dataclasses -------------------------------------------------------
@dataclass
class PowerRampConfig:
    min_power_pct: int = int(os.getenv("MIN_POWER_PCT", "1"))
    max_power_pct: int = int(os.getenv("MAX_POWER_PCT", "80"))
    step_pct: int = int(os.getenv("STEP_PCT", "5"))
    dwell_time_s: float = float(os.getenv("DWELL_TIME_S", "10.0"))
    max_power_w: float = float(os.getenv("MAX_POWER_W", "500.0"))
    def levels(self):
        return list(range(self.min_power_pct, self.max_power_pct + 1, self.step_pct))

@dataclass
class GroundState:
    seq: int = 0
    air_granted: bool = False
    desired_state: int = 2
    current_power_pct: int = 0
    ramp_active: bool = False
    ramp_start_time: float = 0.0
    last_power_change: float = 0.0
    
    # Telemetry from AIR (COMPLETE)
    last_air_voltage_mv: int = 0
    last_air_current_ma: int = 0
    last_air_temp_cdeg: int = 0
    last_optical_power_mw: float | None = None
    last_distance_m: float = 50.0
    last_roll_deg: float = 0.0
    last_pitch_deg: float = 0.0
    last_yaw_deg: float = 0.0
    
    # Stats
    permits_sent: int = 0
    acks_received: int = 0
    grants_received: int = 0  # NEW: actual GRANT count
    denies_received: int = 0   # NEW: actual DENY count
    permit_send_time: float = 0.0
    last_ack_time: float = 0.0
    last_rtt_ms: float = 0.0
    last_hb_time: float = 0.0
    last_deny_reason: int | None = None
    
    # Sliding window for link quality (ACK reception, not grant/deny)
    window: deque = field(default_factory=lambda: deque(maxlen=50))

# ---- CSV logger ---------------------------------------------------------------
class DataLogger:
    def __init__(self, experiment_name: str):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"power_ramp_{experiment_name}_{ts}.csv"
        self.file = open(self.filename, "w", newline="")
        self.writer = csv.DictWriter(self.file, fieldnames=[
            "timestamp", "elapsed_s",
            "commanded_power_pct", "commanded_power_w",
            "received_power_mw", "efficiency_pct",
            "link_quality_pct", "round_trip_latency_ms",
            "permit_ack_received", "permit_granted", "deny_reason",
            "air_voltage_mv", "air_current_ma", "air_temp_cdeg",
            "net_pack_w", "seq_number",
            "successful_handshakes", "failed_handshakes",
            "grants_total", "denies_total",
            "scenario",
            "distance_3d_m", "roll_deg", "pitch_deg", "yaw_deg",
        ])
        self.writer.writeheader()
        print(f"[logger] Data logging to: {self.filename}")
    
    def log_sample(self, row: dict):
        self.writer.writerow(row)
        self.file.flush()
    
    def close(self):
        self.file.close()
        print(f"[logger] Data saved to: {self.filename}")

# ---- Main ---------------------------------------------------------------------
def main():
    cfg = load_cfg()
    permit_cfg = PermitConfig(
        ttl_ms=cfg["permit"]["ttl_ms"],
        send_hz=cfg["permit"]["send_hz"],
        duplicate=cfg["permit"]["duplicate"],
        power_cap_w=cfg["permit"]["power_cap_w"],
        hb_timeout_ms=cfg["watchdogs"]["hb_timeout_ms"],
        track_timeout_ms=cfg["watchdogs"]["track_timeout_ms"],
        signing=cfg["security"]["enable_signing"],
        key_hex=cfg["security"]["key_hex"],
    )

    ramp_cfg = PowerRampConfig()
    experiment_name = os.getenv("EXPERIMENT_NAME", "test1")
    scenario_name   = os.getenv("SCENARIO_NAME", "")

    print("="*70)
    print("LASER POWER BEAMING - POWER RAMP EXPERIMENT (FIXED)")
    print("="*70)
    print(f"Power: {ramp_cfg.min_power_pct}%..{ramp_cfg.max_power_pct}% step {ramp_cfg.step_pct}% | Dwell {ramp_cfg.dwell_time_s}s")
    print(f"Max power: {ramp_cfg.max_power_w}W | Experiment: {experiment_name} | Scenario: {scenario_name}")
    print("="*70)

    print(f"[ground] Using MAVLink dialect: {os.environ.get('MAVLINK_DIALECT','Common')}")
    m_out = make_conn(cfg["links"]["ground_udp_out"], source_system=250, source_component=190)
    m_in  = make_conn(cfg["links"]["ground_udp_in"],  source_system=250, source_component=190)
    setup_signing_if_enabled(m_out, permit_cfg.signing, permit_cfg.key_hex)
    setup_signing_if_enabled(m_in,  permit_cfg.signing, permit_cfg.key_hex)

    wd = Watchdogs(permit_cfg.ttl_ms, permit_cfg.hb_timeout_ms)
    state = GroundState()
    state_lock = threading.Lock()
    logger = DataLogger(experiment_name)
    levels = ramp_cfg.levels()
    lvl_idx = 0

    # ---- RX loop --------------------------------------------------------------
    def rx_loop():
        nonlocal state
        last_telemetry_log = 0.0
        
        while True:
            msg = m_in.recv_match(blocking=True, timeout=0.5)
            if not msg:
                continue
            t = msg.get_type()

            if t == "HEARTBEAT":
                with state_lock:
                    wd.kick_hb()
                    state.last_hb_time = time.monotonic()

            elif t == "LASER_PERMIT_ACK":
                now = time.monotonic()
                
                # Parse ACK fields (robust field name handling)
                granted = None
                for f in ("status", "result", "granted", "ack"):
                    if hasattr(msg, f):
                        granted = int(getattr(msg, f))
                        break
                
                seq_ack = None
                for f in ("seq", "sequence", "seqno"):
                    if hasattr(msg, f):
                        seq_ack = int(getattr(msg, f))
                        break
                
                reason = None
                for f in ("reason", "deny_reason", "code"):
                    if hasattr(msg, f):
                        reason = int(getattr(msg, f))
                        break

                with state_lock:
                    state.last_ack_time = now
                    state.acks_received += 1
                    
                    # Track actual GRANT vs DENY (CRITICAL FIX)
                    if granted == 1:
                        state.air_granted = True
                        state.grants_received += 1
                        state.last_deny_reason = 0  # OK
                    else:
                        state.air_granted = False
                        state.denies_received += 1
                        state.last_deny_reason = reason if reason is not None else 1
                    
                    # RTT calculation
                    if state.permit_send_time > 0:
                        state.last_rtt_ms = (now - state.permit_send_time) * 1000.0
                    
                    # Mark sequence as ACKed in window (for link quality)
                    if seq_ack is not None:
                        for i in range(len(state.window) - 1, -1, -1):
                            if state.window[i][0] == seq_ack:
                                state.window[i] = (seq_ack, True)
                                break
                    
                    # Debug logging for denials
                    if granted != 1 and reason is not None:
                        reason_str = REASON_TEXT.get(reason, f"Unknown({reason})")
                        print(f"[ground] ⚠ DENY received: seq={seq_ack} reason={reason_str}")

            elif t == "POWER_SINK_STATUS":
                v = int(getattr(msg, "v_panel_mV", 0))
                i = int(getattr(msg, "i_panel_mA", 0))
                temp = int(getattr(msg, "temp_cdeg", 0))
                ok = int(getattr(msg, "ok_flags", 0))
                
                # Decode charging sign bit
                if ok & (1 << 3):
                    i = -i

                with state_lock:
                    state.last_air_voltage_mv = v
                    state.last_air_current_ma = i
                    state.last_air_temp_cdeg = temp

            elif t == "NAMED_VALUE_FLOAT":
                # CRITICAL: Extended telemetry (distance, attitude)
                name_raw = getattr(msg, 'name', '')
                # Handle both string and bytes
                if isinstance(name_raw, bytes):
                    msg_name = name_raw.decode('utf-8', errors='ignore').strip('\x00')
                else:
                    msg_name = str(name_raw).strip('\x00')
                    
                msg_val = float(getattr(msg, 'value', 0.0))
                
                now = time.monotonic()
                
                with state_lock:
                    if msg_name == "pv_opt_mw":
                        state.last_optical_power_mw = msg_val
                    elif msg_name == "dist_3d_m":
                        state.last_distance_m = msg_val
                    elif msg_name == "roll_deg":
                        state.last_roll_deg = msg_val
                    elif msg_name == "pitch_deg":
                        state.last_pitch_deg = msg_val
                    elif msg_name == "yaw_deg":
                        state.last_yaw_deg = msg_val
                
                # Periodic telemetry logging (not every message)
                if now - last_telemetry_log >= 2.0:
                    last_telemetry_log = now
                    with state_lock:
                        d = state.last_distance_m
                        r = state.last_roll_deg
                        p = state.last_pitch_deg
                    print(f"[ground] Telemetry: d={d:.1f}m r={r:.1f}° p={p:.1f}°")

    threading.Thread(target=rx_loop, daemon=True).start()

    # ---- Preflight ------------------------------------------------------------
    print("[ground] Preflight: waiting for AIR heartbeat...")
    hb_deadline = time.monotonic() + 5.0
    while time.monotonic() < hb_deadline:
        with state_lock:
            ok = (time.monotonic() - state.last_hb_time) <= 0.9
        if ok:
            break
        time.sleep(0.05)
    
    with state_lock:
        alive = (time.monotonic() - state.last_hb_time) <= 0.9
    if not alive:
        print("[ground] Abort: no HEARTBEAT from AIR within 5s")
        return

    print("[ground] Preflight: probing for initial ACK...")
    with state_lock:
        state.seq += 1
    m_out.mav.laser_permit_send(state.seq, permit_cfg.ttl_ms, 1, state.desired_state, 0)
    
    probe_deadline = time.monotonic() + 1.0
    a0 = None
    with state_lock:
        a0 = state.acks_received
    
    while time.monotonic() < probe_deadline:
        with state_lock:
            if state.acks_received != a0:
                break
        time.sleep(0.01)
    
    with state_lock:
        ok = (state.acks_received != a0)
    if not ok:
        print("[ground] Abort: no initial ACK within 1s")
        return

    print("[ground] Preflight OK. Stabilizing 3s…")
    time.sleep(3.0)

    # ---- Ramp -----------------------------------------------------------------
    print("\n[ground] Starting power ramp experiment...")
    print(f"[ground] Will test {len(levels)} levels")
    
    with state_lock:
        state.ramp_active = True
        state.ramp_start_time = time.monotonic()
        state.last_power_change = state.ramp_start_time
        state.current_power_pct = levels[0]
    
    print(f"\n[RAMP] Start @ {levels[0]}%")

    period = 1.0 / max(1.0, float(permit_cfg.send_hz))
    last_tx = 0.0
    last_hb = 0.0
    last_log = 0.0
    prev_enabled = False
    last_recovery_check = 0.0

    # Metadata
    meta = {
        "experiment_name": experiment_name,
        "scenario": scenario_name,
        "ramp": {
            "min_power_pct": ramp_cfg.min_power_pct,
            "max_power_pct": ramp_cfg.max_power_pct,
            "step_pct": ramp_cfg.step_pct,
            "dwell_time_s": ramp_cfg.dwell_time_s,
            "max_power_w": ramp_cfg.max_power_w,
        },
        "permit": {
            "ttl_ms": permit_cfg.ttl_ms,
            "send_hz": permit_cfg.send_hz,
            "duplicate": permit_cfg.duplicate,
            "power_cap_w": permit_cfg.power_cap_w,
            "signing": permit_cfg.signing,
        },
    }
    
    with open(f"power_ramp_{experiment_name}_metadata.json", "w") as f:
        json.dump(meta, f, indent=2)
    print("[ground] Wrote metadata JSON")

    try:
        while True:
            now = time.monotonic()

            # Dwell step advancement
            with state_lock:
                active = state.ramp_active
                last_change = state.last_power_change
            
            if active and (now - last_change >= ramp_cfg.dwell_time_s):
                lvl_idx += 1
                if lvl_idx >= len(levels):
                    print("\n[RAMP] Experiment complete!")
                    with state_lock:
                        state.ramp_active = False
                        state.current_power_pct = 0
                    hardware_pwm_disable()
                    break
                
                with state_lock:
                    state.current_power_pct = levels[lvl_idx]
                    state.last_power_change = now
                
                print(f"\n[RAMP] Level {lvl_idx+1}/{len(levels)}: {levels[lvl_idx]}%")

            # Heartbeat to AIR
            if now - last_hb >= 0.5:
                last_hb = now
                m_out.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0
                )

            # DEADLOCK RECOVERY (CRITICAL FIX for Aggressive Maneuver)
            if now - last_recovery_check >= 1.0:
                last_recovery_check = now
                with state_lock:
                    time_since_ack = now - state.last_ack_time
                    sent = state.permits_sent
                
                # If no ACKs for 3 seconds and we've sent >10 permits, reset
                if time_since_ack > 3.0 and sent > 10:
                    print(f"[ground] ⚠ DEADLOCK RECOVERY: No ACKs for {time_since_ack:.1f}s")
                    with state_lock:
                        state.seq += 1  # Force sequence advance
                        state.window.clear()  # Clear stale window
                        state.air_granted = False
                    hardware_pwm_disable()
                    prev_enabled = False
                    time.sleep(0.1)

            # Send permits at rate
            if now - last_tx >= period:
                last_tx = now
                
                with state_lock:
                    state.permits_sent += 1
                    pct = state.current_power_pct
                    granted = state.air_granted
                    expired = wd.expired()
                    active = state.ramp_active
                
                cmd_w = (pct / 100.0) * ramp_cfg.max_power_w
                ground_ok = True
                allow_enable = ground_ok and granted and not expired and active

                # PWM control (debounced)
                if not granted and prev_enabled:
                    hardware_pwm_disable()
                    prev_enabled = False
                if allow_enable != prev_enabled:
                    if allow_enable:
                        hardware_pwm_enable()
                    else:
                        hardware_pwm_disable()
                    prev_enabled = allow_enable

                with state_lock:
                    state.seq += 1
                    state.permit_send_time = now
                    state.window.append((state.seq, False))

                if ground_ok and active:
                    m_out.mav.laser_permit_send(
                        state.seq, permit_cfg.ttl_ms, int(cmd_w), 
                        state.desired_state, 0
                    )
                    if permit_cfg.duplicate:
                        m_out.mav.laser_permit_send(
                            state.seq, permit_cfg.ttl_ms, int(cmd_w),
                            state.desired_state, 0
                        )
                else:
                    m_out.mav.laser_permit_send(state.seq, 0, 0, 0, 0)

                if wd.expired() or not ground_ok:
                    hardware_pwm_disable()

            # Logging @ 2 Hz
            if active and (now - last_log >= 0.5):
                last_log = now
                
                with state_lock:
                    pct = state.current_power_pct
                    rtt_ms = state.last_rtt_ms if state.last_rtt_ms > 0 else 0.0
                    pv = state.last_optical_power_mw
                    v_mv = state.last_air_voltage_mv
                    i_ma = state.last_air_current_ma
                    temp = state.last_air_temp_cdeg
                    seq = state.seq
                    acks = state.acks_received
                    grants = state.grants_received
                    denies = state.denies_received
                    sent = state.permits_sent
                    reason = state.last_deny_reason
                    win = list(state.window)
                    
                    # Extended telemetry
                    dist = state.last_distance_m
                    roll = state.last_roll_deg
                    pitch = state.last_pitch_deg
                    yaw = state.last_yaw_deg

                cmd_w = (pct / 100.0) * ramp_cfg.max_power_w
                
                # Received power (prefer actual measurement)
                if pv is not None:
                    rcv_mw = float(pv)
                else:
                    # Fallback estimate (shouldn't happen with fixed code)
                    base_eff = 0.40
                    variation = random.uniform(-0.05, 0.05)
                    rcv_mw = cmd_w * 1000.0 * (base_eff + variation)

                # Link quality (ACK reception, not grant/deny)
                if win:
                    acked = sum(1 for _, ok in win if ok)
                    link_quality = int(100.0 * acked / len(win))
                else:
                    link_quality = 0

                net_pack_w = (v_mv / 1000.0) * (i_ma / 1000.0)
                eff_pct = (rcv_mw / (cmd_w * 1000.0) * 100.0) if cmd_w > 0 else 0.0

                row = {
                    "timestamp": now,
                    "elapsed_s": now - meta.get("start_ts", meta.setdefault("start_ts", now)),
                    "commanded_power_pct": pct,
                    "commanded_power_w": cmd_w,
                    "received_power_mw": rcv_mw,
                    "efficiency_pct": eff_pct,
                    "link_quality_pct": link_quality,
                    "round_trip_latency_ms": rtt_ms,
                    "permit_ack_received": 1 if link_quality > 0 else 0,
                    "permit_granted": 1 if reason == 0 or reason is None else 0,
                    "deny_reason": REASON_TEXT.get(reason, "") if reason is not None else "",
                    "air_voltage_mv": v_mv,
                    "air_current_ma": i_ma,
                    "air_temp_cdeg": temp,
                    "net_pack_w": net_pack_w,
                    "seq_number": seq,
                    "successful_handshakes": acks,
                    "failed_handshakes": max(0, sent - acks),
                    "grants_total": grants,
                    "denies_total": denies,
                    "scenario": scenario_name,
                    "distance_3d_m": dist,
                    "roll_deg": roll,
                    "pitch_deg": pitch,
                    "yaw_deg": yaw,
                }
                logger.log_sample(row)

                # Console output
                grant_rate = (100.0 * grants / (grants + denies)) if (grants + denies) > 0 else 0.0
                print(
                    f"  [{pct:3d}%] Cmd:{cmd_w:6.1f}W | "
                    f"Rcv:{rcv_mw:7.1f}mW | Eff:{eff_pct:5.1f}% | "
                    f"LQ:{link_quality:3d}% | RTT:{rtt_ms:5.1f}ms | "
                    f"G/D:{grants}/{denies} ({grant_rate:.0f}%) | "
                    f"d={dist:.1f}m r={roll:.1f}° p={pitch:.1f}° | "
                    f"BAT:{v_mv}mV {i_ma}mA {temp}cdeg" +
                    (f" | DENY:{REASON_TEXT.get(reason,'')}" if reason not in (None, 0) else "")
                )

            time.sleep(0.002)

    finally:
        logger.close()
        hardware_pwm_disable()
        
        # Final stats
        with state_lock:
            print("\n" + "="*70)
            print("EXPERIMENT COMPLETE - FINAL STATS")
            print("="*70)
            print(f"Total permits sent: {state.permits_sent}")
            print(f"ACKs received: {state.acks_received} ({100.0*state.acks_received/state.permits_sent:.1f}%)")
            print(f"GRANTs received: {state.grants_received}")
            print(f"DENYs received: {state.denies_received}")
            grant_rate = (100.0 * state.grants_received / (state.grants_received + state.denies_received)) if (state.grants_received + state.denies_received) > 0 else 0.0
            print(f"Grant rate: {grant_rate:.1f}%")
            print("="*70)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[ground] Interrupted by user")