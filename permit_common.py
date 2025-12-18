#!/usr/bin/env python3
# permit_common.py

import os
import sys
import time
import logging
from dataclasses import dataclass

# -------------------------------------------------------------------
# Dialect setup: MUST be done BEFORE importing pymavlink
# -------------------------------------------------------------------
# We default to MAVLink 2.0 + your custom dialect. If the user already
# exported these in the shell, we don't overwrite them.
os.environ.setdefault("MAVLINK20", "1")
os.environ.setdefault("MAVLINK_DIALECT", "laser_safety")

from pymavlink import mavutil  # import AFTER env vars are set


# -------------------------------------------------------------------
# Logging
# -------------------------------------------------------------------
log = logging.getLogger("permit")
log.setLevel(logging.INFO)
_h = logging.StreamHandler(sys.stdout)
_h.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
if not any(isinstance(h, logging.StreamHandler) for h in log.handlers):
    log.addHandler(_h)

# Optional sanity print so you can see the active dialect & custom msg
try:
    has_custom = hasattr(mavutil.mavlink, "MAVLINK_MSG_ID_LASER_PERMIT")
    print(f"[permit_common] Dialect: {os.environ.get('MAVLINK_DIALECT','<unset>')} "
          f"| LASER_PERMIT known: {has_custom}")
except Exception:
    pass


# -------------------------------------------------------------------
# Config dataclass
# -------------------------------------------------------------------
@dataclass
class PermitConfig:
    ttl_ms: int = 300
    send_hz: int = 10
    duplicate: bool = True
    power_cap_w: int = 20
    hb_timeout_ms: int = 2000
    track_timeout_ms: int = 150
    signing: bool = False
    key_hex: str = ""


# -------------------------------------------------------------------
# Config file loader
# -------------------------------------------------------------------
def load_cfg(path: str = "safety_config.yaml") -> dict:
    import yaml
    with open(path, "r") as f:
        return yaml.safe_load(f)


# -------------------------------------------------------------------
# MAVLink connection helper
# -------------------------------------------------------------------
def make_conn(endpoint: str, source_system: int = 245, source_component: int = 191):
    """
    Create a MAVLink connection using pymavlink.
    Assumes the dialect is already selected via environment vars.
    """
    m = mavutil.mavlink_connection(
        endpoint,
        source_system=source_system,
        source_component=source_component,
    )
    print(f"[permit_common] Created connection: {endpoint}")
    return m


# -------------------------------------------------------------------
# Signing helper (fail closed if requested but cannot enable)
# -------------------------------------------------------------------
def setup_signing_if_enabled(m, enable: bool, key_hex: str):
    if not enable:
        log.info("Signing disabled")
        return
    try:
        key = bytes.fromhex(key_hex)
    except ValueError:
        log.error("Signing key is not valid hex; refusing to proceed")
        raise

    try:
        # Preferred API on newer pymavlink
        m.setup_signing(key)
        # Ensure outgoing signing is definitely on
        if hasattr(m, "mav") and hasattr(m.mav, "signing"):
            m.mav.signing.sign_outgoing = True
        log.info("MAVLink2 signing enabled")
    except Exception as e:
        # Fallback manual wiring (older pymavlink)
        try:
            signer = mavutil.mavlink.MAVLinkSigning(key)
            signer.link_id = 1  # non-zero recommended
            signer.sign_outgoing = True
            m.mav.signing = signer
            log.info("MAVLink2 signing enabled (manual)")
        except Exception as ee:
            # FAIL CLOSED: cut hardware path and raise
            log.error(f"Failed to enable signing: {e or ee}")
            hardware_pwm_disable()
            raise


# -------------------------------------------------------------------
# Watchdogs (permit + heartbeat)
# -------------------------------------------------------------------
class Watchdogs:
    def __init__(self, permit_ttl_ms: int, hb_timeout_ms: int):
        self.permit_expire = 0.0
        self.hb_expire = 0.0
        self.permit_ttl = permit_ttl_ms / 1000.0
        self.hb_ttl = hb_timeout_ms / 1000.0
        self.seen_permit = False
        self.seen_hb = False

    def kick_permit(self):
        self.permit_expire = time.monotonic() + self.permit_ttl
        self.seen_permit = True

    def kick_hb(self):
        self.hb_expire = time.monotonic() + self.hb_ttl
        self.seen_hb = True

    def expired(self):
        now = time.monotonic()
        permit_bad = self.seen_permit and (now > self.permit_expire)
        hb_bad = self.seen_hb and (now > self.hb_expire)
        return permit_bad or hb_bad


    def permit_expired(self) -> bool:
        return time.monotonic() > self.permit_expire

    def hb_expired(self) -> bool:
        return time.monotonic() > self.hb_expire


# -------------------------------------------------------------------
# Hardware stubs (replace with your GPIO/relay code)
# -------------------------------------------------------------------
def hardware_pwm_disable():
    # Placeholder for PSU relay PWM drop; replace with GPIO code on your platform
    log.warning("[HARDWARE] PWM -> DISABLE (stub)")


def hardware_pwm_enable():
    # Placeholder; ensure you only call this when all guards are satisfied
    log.info("[HARDWARE] PWM -> ENABLE (stub)")