#!/usr/bin/env python3
"""
Air (Companion) node for Laser Power Beaming — COMPLETELY FIXED
- Attitude-dependent optical power sensor (CORRECT implementation)
- Distance tracking with proper geometric losses
- Publishes POWER_SINK_STATUS + extended telemetry
- Grants/DENYs LASER_PERMIT with precise reason codes
"""

import os, time, threading, random, math
from dataclasses import dataclass
from permit_common import (
    load_cfg, PermitConfig, make_conn, setup_signing_if_enabled,
    Watchdogs, hardware_pwm_disable, hardware_pwm_enable
)
from pymavlink import mavutil


# -------- Env & constants -----------------------------------------------------
os.environ.setdefault("MAVLINK20", "1")
os.environ.setdefault("MAVLINK_DIALECT", "laser_safety")
USE_PX4     = os.getenv("USE_PX4", "1") == "1"
PX4_TX_PORT = int(os.getenv("PX4_TX_PORT", "14780"))
PX4_RX_PORT = int(os.getenv("PX4_RX_PORT", "14740"))
SIM_SEED    = int(os.getenv("SIM_SEED", "12345"))
random.seed(SIM_SEED)

def clamp_u16(x: int) -> int: return max(0, min(int(x), 65535))
SIGN_CHARGING_BIT = 1 << 3

# DENY reason map
REASON_OK            = 0
REASON_GATES_FAILED  = 1
REASON_SEQ_WINDOW    = 2
REASON_TTL_EXPIRED   = 3
REASON_PX4_NOT_OK    = 4
REASON_LOCAL_HEALTH  = 5
REASON_GROUND_INTENT = 6

def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat/2.0)**2 + math.cos(math.radians(lat1))*math.cos(math.radians(lat2))*math.sin(dlon/2.0)**2
    return 2.0 * R * math.asin(math.sqrt(a))

# -------- State ----------------------------------------------------------------
@dataclass
class AirState:
    last_seq: int = -1
    local_ok: bool = True
    granted: bool = False
    last_commanded_power_w: float = 0.0
    measured_optical_power_mw: float = 0.0
    # Battery model
    battery_soc: float = 0.75
    battery_voltage_mv: int = 15200
    battery_current_ma: int = 0
    temperature_cdeg: int = 2500
    # Telemetry
    distance_m: float = 50.0
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0

# -------- COMPLETELY FIXED Optical power sensor -------------------------------
class OpticalPowerSensor:
    """
    REALISTIC optical sensor with attitude-dependent pointing loss.
    
    CRITICAL FIXES:
    1. Pointing efficiency correctly depends on roll/pitch
    2. Outside ±12° cone: exponential falloff (beam misses receiver)
    3. Geometric losses scale with distance squared
    4. No orphaned methods - all physics in measure_power()
    """
    def __init__(self):
        self.base_efficiency = 0.40  # PV panel conversion efficiency
        self.noise_sigma_mw = 5.0
        self.beam_divergence_rad = 0.001  # 1 mrad full angle
        self.receiver_area_m2 = 0.01  # 10cm x 10cm panel (100 cm²)
        self.cone_limit_deg = 12.0  # Safety gate threshold

    def measure_power(self, commanded_power_w: float, distance_m: float, 
                      roll_deg: float = 0.0, pitch_deg: float = 0.0) -> float:
        """
        Measure received optical power with ALL realistic losses.
        
        Physics model:
        - Atmospheric transmission: Beer-Lambert law (very clear air)
        - Geometric spreading: 1/r² with beam divergence
        - Pointing loss: Attitude-dependent (CRITICAL FIX)
        - Conversion efficiency: PV panel (40% baseline)
        
        Args:
            commanded_power_w: Laser output power (W)
            distance_m: 3D distance to ground station (m)
            roll_deg: Platform roll angle (deg)
            pitch_deg: Platform pitch angle (deg)
        
        Returns:
            Received optical power (mW) after all losses
        """
        if commanded_power_w <= 0:
            return 0.0
        
        # 1. ATMOSPHERIC TRANSMISSION (Beer-Lambert)
        attenuation_db_per_km = 0.1  # Very clear conditions
        attenuation_db = attenuation_db_per_km * (distance_m / 1000.0)
        T_atm = 10 ** (-attenuation_db / 10.0)
        # Scintillation (turbulence-induced fading)
        scint = random.uniform(0.97, 1.0)
        T_atm *= scint
        
        # 2. GEOMETRIC SPREADING LOSS
        # Beam expands with distance: diameter = 2 * distance * divergence
        spot_diameter_m = max(1e-3, 2.0 * distance_m * self.beam_divergence_rad)
        spot_area_m2 = math.pi * (spot_diameter_m / 2.0) ** 2
        # Fraction of beam captured by receiver
        geometric_factor = min(1.0, self.receiver_area_m2 / spot_area_m2)
        
        # 3. POINTING LOSS (ATTITUDE-DEPENDENT) - CRITICAL FIX
        # Total attitude error (Pythagorean combination)
        attitude_error_deg = math.sqrt(roll_deg**2 + pitch_deg**2)
        
        if attitude_error_deg <= self.cone_limit_deg:
            # INSIDE CONE: Linear degradation
            # At 0°: 100% pointing efficiency
            # At 12°: 70% pointing efficiency (edge of cone)
            base_pointing = 1.0 - (attitude_error_deg / self.cone_limit_deg) * 0.30
            # Add small random jitter (±5%) to simulate vibration
            pointing_factor = base_pointing * random.uniform(0.95, 1.05)
        else:
            # OUTSIDE CONE: Exponential falloff (beam misses receiver)
            # At 12°: 70% (cone edge)
            # At 20°: ~20% (mostly missing)
            # At 30°: ~5% (almost entirely missing)
            excess_deg = attitude_error_deg - self.cone_limit_deg
            pointing_factor = 0.70 * math.exp(-excess_deg / 8.0)
            # Larger jitter when outside cone (±10%)
            pointing_factor *= random.uniform(0.90, 1.10)
        
        # Clamp to valid range
        pointing_factor = max(0.0, min(1.0, pointing_factor))
        
        # 4. PV CONVERSION EFFICIENCY
        # Baseline 40% with small manufacturing variation (±2%)
        conversion_efficiency = self.base_efficiency * random.uniform(0.98, 1.02)
        
        # 5. COMBINED POWER TRANSFER
        received_mw = (commanded_power_w * 1000.0 *  # Convert W to mW
                       T_atm *                        # Atmospheric loss
                       geometric_factor *             # Beam spreading
                       pointing_factor *              # Attitude alignment
                       conversion_efficiency)         # PV efficiency
        
        # 6. SENSOR NOISE
        noise = random.gauss(0.0, self.noise_sigma_mw)
        
        return max(0.0, received_mw + noise)

# -------- PX4 gate & position --------------------------------------------------
PX4_ALT_LOG_EVERY_S = 0.5
PX4_BAT_LOG_EVERY_S = 1.0
class PX4Gate:
    def __init__(self):
        self.enabled = USE_PX4
        self.armed = False
        self.ekf_ok = True
        self.last_hb = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        # Position
        self.has_home = False
        self._home_ready_samples = 0
        self.home_lat = 0.0
        self.home_lon = 0.0
        self.last_lat = None
        self.last_lon = None
        self.rel_alt_m = 0.0
        # MAVLink conns
        self.tx = None
        self.rx = None
        # Debounce
        self._last_gate_tuple = None

    def hb_recent(self) -> bool:
        return (time.monotonic() - self.last_hb) < 1.2

    def ok(self, cone_deg: float = 12.0) -> bool:
        if not self.enabled:
            return True
        inside_cone = abs(self.roll) <= math.radians(cone_deg) and abs(self.pitch) <= math.radians(cone_deg)
        tup = (self.hb_recent(), self.armed, self.ekf_ok, inside_cone)
        if tup != self._last_gate_tuple:
            roll_d = math.degrees(self.roll)
            pitch_d = math.degrees(self.pitch)
            print(f"[air] PX4 gate: hb={int(tup[0])} armed={int(tup[1])} ekf={int(tup[2])} "
                  f"cone={int(tup[3])} (r={roll_d:.1f}° p={pitch_d:.1f}°)")
            self._last_gate_tuple = tup
        return all(tup)

    def qgc_note(self, txt: str):
        try:
            if self.tx:
                self.tx.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, txt.encode()[:50])
        except Exception:
            pass

    def qgc_val(self, name: str, val: float):
        try:
            if self.tx:
                self.tx.mav.named_value_float_send(int(time.time()), name.encode()[:10], float(val))
        except Exception:
            pass

    def distance_m(self) -> float:
        if not self.has_home or self.last_lat is None or self.last_lon is None:
            return 50.0
        dh = haversine_m(self.home_lat, self.home_lon, self.last_lat, self.last_lon)
        dz = max(0.0, float(self.rel_alt_m))
        return math.sqrt(dh*dh + dz*dz)

px4 = PX4Gate()

def start_px4_links():
    if not px4.enabled:
        print("[air] PX4/QGC side-band: DISABLED (USE_PX4=0)")
        return
    try:
        px4.tx = mavutil.mavlink_connection(
            f"udpout:127.0.0.1:{PX4_TX_PORT}", source_system=246, source_component=192
        )
        px4.rx = mavutil.mavlink_connection(
            f"udpin:0.0.0.0:{PX4_RX_PORT}", source_system=246, source_component=192
        )
        print(f"[air] PX4/QGC side-band: TX→{PX4_TX_PORT} RX←{PX4_RX_PORT}")

        def hb_loop():
            while True:
                try:
                    px4.tx.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                        mavutil.mavlink.MAV_STATE_ACTIVE
                    )
                except Exception:
                    pass
                time.sleep(1.0)

        def rx_loop():
            last_alt_log = 0.0
            last_bat_log = 0.0
            while True:
                msg = px4.rx.recv_match(blocking=True, timeout=0.3)
                if not msg:
                    continue
                t = msg.get_type()
                now = time.monotonic()
                if t == 'HEARTBEAT':
                    px4.last_hb = time.monotonic()
                    base_mode = getattr(msg, 'base_mode', 0)
                    px4.armed = bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                elif t == 'ATTITUDE':
                    px4.roll = getattr(msg, 'roll', 0.0)
                    px4.pitch = getattr(msg, 'pitch', 0.0)
                    px4.yaw = getattr(msg, 'yaw', 0.0)
                elif t == 'ESTIMATOR_STATUS':
                    flags = getattr(msg, 'flags', 0)
                    px4.ekf_ok = bool(flags & (1 << 0))
                elif t == 'GLOBAL_POSITION_INT':
                    lat = getattr(msg, 'lat', None)
                    lon = getattr(msg, 'lon', None)
                    ralt = getattr(msg, 'relative_alt', None)
                    if lat is not None and lon is not None:
                        px4.last_lat = lat / 1e7
                        px4.last_lon = lon / 1e7
                        if not px4.has_home and px4.last_lat != 0.0 and px4.last_lon != 0.0:
                            px4._home_ready_samples += 1
                            if px4._home_ready_samples >= 5:
                                px4.home_lat = px4.last_lat
                                px4.home_lon = px4.last_lon
                                px4.has_home = True
                                print(f"[air] Home set: {px4.home_lat:.6f}, {px4.home_lon:.6f}")
                    if ralt is not None:
                        # GLOBAL_POSITION_INT.relative_alt is in millimeters, convert to meters
                        px4.rel_alt_m = ralt / 1000.0
                        if now - last_alt_log >= PX4_ALT_LOG_EVERY_S:
                            last_alt_log = now
                            print(f"[air] PX4 ALT rel={px4.rel_alt_m:.2f}m")

                # Optional: read PX4's own battery to surface in UI (Step 2 below)
                elif t == 'BATTERY_STATUS':
                    # Classic fields: voltages[0] in mV, current_battery in cA (10 mA units), remaining (0..100 or 255)
                    voltages = getattr(msg, 'voltages', None)
                    vbatt_mv = int(voltages[0]) if voltages and voltages[0] < 65535 else -1
                    current_cA = int(getattr(msg, 'current_battery', -1))
                    remaining = int(getattr(msg, 'battery_remaining', -1))
                    if vbatt_mv >= 0 and now - last_bat_log >= PX4_BAT_LOG_EVERY_S:
                        last_bat_log = now
                        # Convert current from cA (10 mA units) to mA
                        ibatt_ma = current_cA * 10 if current_cA != -1 else -1
                        rem_pct = remaining if (0 <= remaining <= 100) else -1
                        print(f"[air] PX4 BAT V={vbatt_mv}mV I={ibatt_ma}mA rem={rem_pct}%")
                

        threading.Thread(target=hb_loop, daemon=True).start()
        threading.Thread(target=rx_loop, daemon=True).start()
    except Exception as e:
        print(f"[air] PX4 side-band failed: {e}")
        px4.enabled = False

# -------- Main -----------------------------------------------------------------
def main():
    cfg = load_cfg()
    permit_cfg = PermitConfig(
        ttl_ms=cfg['permit']['ttl_ms'],
        send_hz=cfg['permit']['send_hz'],
        duplicate=cfg['permit']['duplicate'],
        power_cap_w=cfg['permit']['power_cap_w'],
        hb_timeout_ms=cfg['watchdogs']['hb_timeout_ms'],
        track_timeout_ms=cfg['watchdogs']['track_timeout_ms'],
        signing=cfg['security']['enable_signing'],
        key_hex=cfg['security']['key_hex'],
    )

    print(f"[air] Using MAVLink dialect: {os.environ.get('MAVLINK_DIALECT', 'Common')}")
    print("[air] Optical power sensor: REALISTIC (attitude + distance dependent)")
    print("[air] Battery model: 68 Wh pack, SOC-based voltage")

    # Link endpoints
    if 'air_serial' in cfg['links'] and cfg['links']['air_serial'] != "disabled":
        endpoint_in = cfg['links']['air_serial']
        endpoint_out = cfg['links']['air_serial']
        print(f"[air] Using ELRS serial: {endpoint_in}")
    else:
        endpoint_in  = cfg['links'].get('air_udp_in', 'udpin:0.0.0.0:14600')
        endpoint_out = 'udpout:127.0.0.1:14560'
        print(f"[air] Using UDP fallback: {endpoint_in}")

    m_in  = make_conn(endpoint_in,  source_system=245, source_component=191)
    m_out = m_in if endpoint_in == endpoint_out else make_conn(endpoint_out, source_system=245, source_component=191)

    setup_signing_if_enabled(m_in,  permit_cfg.signing, permit_cfg.key_hex)
    setup_signing_if_enabled(m_out, permit_cfg.signing, permit_cfg.key_hex)

    start_px4_links()

    wd = Watchdogs(permit_cfg.ttl_ms, permit_cfg.hb_timeout_ms)
    state = AirState()
    sensor = OpticalPowerSensor()
    send_lock = threading.Lock()

    # Debounced PWM control
    pwm_enabled = False
    def set_pwm(enable: bool):
        nonlocal pwm_enabled
        if enable != pwm_enabled:
            if enable:
                print("INFO [HARDWARE] PWM -> ENABLE (stub)")
                hardware_pwm_enable()
            else:
                print("INFO [HARDWARE] PWM -> DISABLE (stub)")
                hardware_pwm_disable()
            pwm_enabled = enable

    def compute_local_ok():
        temp_ok    = state.temperature_cdeg < 5000
        voltage_ok = state.battery_voltage_mv > 12000
        current_ok = abs(state.battery_current_ma) < 20000
        return temp_ok and voltage_ok and current_ok, temp_ok, voltage_ok, current_ok

    # --- Telemetry TX loop ----------------------------------------------------
    def tx_status_loop():
        t_prev = time.monotonic()
        last_hb = 0.0
        PACK_WH = 68.0
        nominal_V = 15.2
        pack_Ah = PACK_WH / nominal_V
        temp_float = 25.0

        while True:
            now = time.monotonic()
            dt = max(0.02, now - t_prev)
            t_prev = now
            t_ms = int((now) * 1000) & 0xFFFFFFFF

            # Ground-facing HB
            if now - last_hb >= 0.5:
                last_hb = now
                with send_lock:
                    m_out.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GENERIC,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0
                    )

            # Battery current model
            if state.measured_optical_power_mw > 1500.0:
                charging_power_w = state.measured_optical_power_mw / 1000.0
                charging_current_a = min(charging_power_w / nominal_V * 0.9, 3.0)
                state.battery_current_ma = -int(charging_current_a * 1000.0)
            else:
                base_draw_a = 10.0 + 2.0 * random.uniform(-1.0, 1.0)
                state.battery_current_ma = int(base_draw_a * 1000.0)

            # SOC integration
            I_a = state.battery_current_ma / 1000.0
            d_soc = -(I_a * (dt / 3600.0)) / pack_Ah
            state.battery_soc = max(0.0, min(1.0, state.battery_soc + d_soc))

            # Voltage model
            ocv = 13.2 + 3.6 * state.battery_soc
            droop = min(0.8, (abs(I_a) / 15.0) * 0.8)
            v = max(12.0, ocv - droop)
            state.battery_voltage_mv = int(v * 1000.0)

            # Temperature
            heat = 0.002 * abs(I_a) + (0.004 if state.measured_optical_power_mw > 5000 else 0.0)
            temp_float += (heat - 0.001*(temp_float - 25.0)) * (dt*50)
            state.temperature_cdeg = int(max(20.0, min(80.0, temp_float)) * 100)

            # Local health + sign bit
            loc_ok, temp_ok, volt_ok, curr_ok = compute_local_ok()
            state.local_ok = loc_ok
            ok_flags = (int(curr_ok) << 0) | (int(volt_ok) << 1) | (int(temp_ok) << 2)
            if state.battery_current_ma < 0:
                ok_flags |= SIGN_CHARGING_BIT

            v_mV = clamp_u16(state.battery_voltage_mv)
            i_mA = clamp_u16(abs(state.battery_current_ma))
            t_cd = clamp_u16(state.temperature_cdeg)
            pv_mw = int(max(0.0, state.measured_optical_power_mw))

            with send_lock:
                # POWER_SINK_STATUS
                m_out.mav.power_sink_status_send(t_ms, v_mV, i_mA, t_cd, ok_flags)
                
                # EXTENDED TELEMETRY via NAMED_VALUE_FLOAT
                m_out.mav.named_value_float_send(int(time.time()), b"pv_opt_mw", float(pv_mw))
                m_out.mav.named_value_float_send(int(time.time()), b"dist_3d_m", float(state.distance_m))
                m_out.mav.named_value_float_send(int(time.time()), b"roll_deg", float(state.roll_deg))
                m_out.mav.named_value_float_send(int(time.time()), b"pitch_deg", float(state.pitch_deg))
                m_out.mav.named_value_float_send(int(time.time()), b"yaw_deg", float(state.yaw_deg))

            time.sleep(0.2)  # 5 Hz

            if px4.tx:
                voltages = [max(0, min(state.battery_voltage_mv, 65535))] + [65535]*9
                current_cA = int(round(state.battery_current_ma / 10.0))  # mA -> cA (10 mA units)
                remaining_pct = int(round(state.battery_soc * 100.0))
                remaining_pct = max(0, min(remaining_pct, 100))
                try:
                    px4.tx.mav.battery_status_send(
                        0,  # battery id
                        mavutil.mavlink.MAV_BATTERY_FUNCTION_ALL,
                        mavutil.mavlink.MAV_BATTERY_TYPE_LIPO,
                        int(state.temperature_cdeg),   # centi-deg C
                        voltages,                      # 10 x uint16 mV
                        current_cA,                    # centi-amps (10 mA)
                        -1,                            # current_consumed mAh unknown
                        -1,                            # energy_consumed unknown
                        remaining_pct                  # %
                    )
                except Exception:
                    pass

    threading.Thread(target=tx_status_loop, daemon=True).start()
    print("[air] RX loop starting...")

    # --- Main RX loop (permits) ----------------------------------------------
    while True:
        msg = m_in.recv_match(blocking=True, timeout=0.5)
        if not msg:
            if USE_PX4 and not px4.ok():
                state.granted = False
                set_pwm(False)
            if wd.expired():
                state.granted = False
                set_pwm(False)
            continue

        t = msg.get_type()

        if t == 'LASER_PERMIT':
            ACCEPT_WINDOW = 10
            seq            = int(getattr(msg, 'seq', 0))
            ttl_ms         = int(getattr(msg, 'ttl_ms', 0))
            commanded_w    = int(getattr(msg, 'power_cap_w', 0))
            desired_state  = int(getattr(msg, 'state_intent', 0))
            ground_enables = ttl_ms > 0 and desired_state >= 2

            # Sequence filter
            if state.last_seq != -1:
                delta = (seq - state.last_seq) & 0xFFFFFFFF
                if delta == 0:
                    continue  # duplicate
                elif not (0 < delta <= ACCEPT_WINDOW):
                    with send_lock:
                        m_out.mav.laser_permit_ack_send(seq, 0, REASON_SEQ_WINDOW)
                    state.last_seq = seq
                    state.granted = False
                    set_pwm(False)
                    continue
            state.last_seq = seq

            # Distance & attitude from PX4
            d = px4.distance_m() if USE_PX4 else 50.0
            state.distance_m = d
            state.roll_deg = math.degrees(px4.roll) if USE_PX4 else 0.0
            state.pitch_deg = math.degrees(px4.pitch) if USE_PX4 else 0.0
            state.yaw_deg = math.degrees(px4.yaw) if USE_PX4 else 0.0

            # CRITICAL: Optical measurement with CORRECT attitude dependency
            state.last_commanded_power_w = float(commanded_w)
            state.measured_optical_power_mw = sensor.measure_power(
                float(commanded_w), d, state.roll_deg, state.pitch_deg
            )

            # Gates
            px4_ok = px4.ok() if USE_PX4 else True
            gates_ok = state.local_ok and px4_ok and ground_enables

            if gates_ok:
                state.granted = True
                wd.kick_permit()
                set_pwm(True)
                with send_lock:
                    m_out.mav.laser_permit_ack_send(seq, 1, REASON_OK)
                
                px4.qgc_note(f"LASER: GRANT {commanded_w}W d={d:.1f}m")
                px4.qgc_val("laser_cmd", float(commanded_w))
                px4.qgc_val("laser_rcv", float(state.measured_optical_power_mw/1000.0))
                eff = (state.measured_optical_power_mw/(1000.0*commanded_w))*100.0 if commanded_w>0 else 0.0
                px4.qgc_val("laser_eff", eff)
                print(f"[air] ✓ GRANT seq={seq} | Cmd:{commanded_w}W | Rcv:{state.measured_optical_power_mw:.1f}mW | "
                      f"Eff:{eff:.1f}% | d={d:.1f}m | r={state.roll_deg:.1f}° p={state.pitch_deg:.1f}°")
            else:
                # Pick precise reason
                if not ground_enables: 
                    reason = REASON_GROUND_INTENT
                elif USE_PX4 and not px4_ok: 
                    reason = REASON_PX4_NOT_OK
                elif not state.local_ok: 
                    reason = REASON_LOCAL_HEALTH
                else: 
                    reason = REASON_GATES_FAILED
                
                state.granted = False
                set_pwm(False)
                with send_lock:
                    m_out.mav.laser_permit_ack_send(seq, 0, reason)
                
                # Enhanced debug logging for DENYs
                if USE_PX4 and reason == REASON_PX4_NOT_OK:
                    att_err = math.sqrt(state.roll_deg**2 + state.pitch_deg**2)
                    print(f"[air] ✗ DENY seq={seq} | PX4_NOT_OK | "
                          f"r={state.roll_deg:.1f}° p={state.pitch_deg:.1f}° | "
                          f"att_err={att_err:.1f}° (cone=12°) | "
                          f"d={d:.1f}m")
                else:
                    print(f"[air] ✗ DENY seq={seq} | reason={reason} | "
                          f"r={state.roll_deg:.1f}° p={state.pitch_deg:.1f}° d={d:.1f}m")

        elif t == 'HEARTBEAT':
            wd.kick_hb()

        elif t == 'LASER_STATE_SET':
            if int(getattr(msg, 'state', 0)) == mavutil.mavlink.LASER_STATE_LASER_SAFE:
                state.granted = False
                set_pwm(False)

        # Watchdog expiry -> SAFE
        if wd.expired():
            state.granted = False
            set_pwm(False)

if __name__ == '__main__':
    main()