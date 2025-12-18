#!/usr/bin/env python3
import os, threading, time, signal, sys, collections
import queue
import heapq
from dataclasses import dataclass
from typing import Any
os.environ.setdefault("MAVLINK20", "1")
os.environ.setdefault("MAVLINK_DIALECT", "laser_safety")
from pymavlink import mavutil
import random

@dataclass(order=True)
class TimedPacket:
    delivery_time: float
    packet: Any = None  # The actual MAVLink message buffer

class AsyncPacketQueue:
    def __init__(self):
        self._queue = []
        self._lock = threading.Lock()
    
    def put(self, packet, delay):
        delivery_time = time.monotonic() + delay
        with self._lock:
            heapq.heappush(self._queue, TimedPacket(delivery_time, packet))
    
    def get_ready(self):
        """Returns list of packets that are ready for delivery"""
        now = time.monotonic()
        ready_packets = []
        
        with self._lock:
            while self._queue and self._queue[0].delivery_time <= now:
                ready_packets.append(heapq.heappop(self._queue).packet)
                
        return ready_packets



UDP_IN  = os.getenv("RELAY_UDP_IN",  "udpin:0.0.0.0:14600")
UDP_OUT = os.getenv("RELAY_UDP_OUT", "udpout:127.0.0.1:14560")
SERIAL  = os.getenv("RELAY_SERIAL",  "/tmp/ELRS_TX,57600")


# Minimize air-link traffic (ELRS-style):
ALLOW_GROUND_TO_AIR = {
    "LASER_PERMIT", "LASER_STATE_SET", "HEARTBEAT", "STATUSTEXT"
}
ALLOW_AIR_TO_GROUND = {
    "LASER_PERMIT_ACK", "POWER_SINK_STATUS", "HEARTBEAT", "STATUSTEXT", "NAMED_VALUE_FLOAT"
}

PRINT_EVERY = 1.0  # seconds

print(f"[mav_relay] UDP_IN={UDP_IN} UDP_OUT={UDP_OUT} SERIAL={SERIAL}")
stop_flag = False

def make_conn(endpoint, sysid=241, compid=191, autoreconnect=True):
    return mavutil.mavlink_connection(endpoint,
                                      source_system=sysid,
                                      source_component=compid,
                                      autoreconnect=autoreconnect)

def safe_write(conn, buf):
    try:
        conn.write(buf)
        return True
    except Exception as e:
        return False

def pump(src, dst, allow, label, counter):
    last_log = 0.0
    PACKET_LOSS_RATE = 0.0  # Disable for testing
    BASE_LATENCY_S = 0.005  # Reduce to 5ms
    JITTER_S = 0.002        # Reduce jitter
    CHECK_INTERVAL = 0.001  # 1ms check interval
    
    packet_queue = AsyncPacketQueue()

    def packet_sender():
        """Separate thread to handle packet delivery timing"""
        while not stop_flag:
            ready_packets = packet_queue.get_ready()
            for packet in ready_packets:
                ok = safe_write(dst, packet)
                if not ok:
                    print(f"[mav_relay] {label}: Failed to write packet")
            time.sleep(CHECK_INTERVAL)

    # Start the sender thread
    sender_thread = threading.Thread(target=packet_sender, daemon=True)
    sender_thread.start()

    # Main receive loop
    while not stop_flag:
        try:
            msg = src.recv_match(blocking=True, timeout=0.1)
        except Exception:
            msg = None

        if not msg:
            continue

        if random.random() < PACKET_LOSS_RATE:
            continue # Simulate a dropped packet
        
        t = msg.get_type()
        if t == "BAD_DATA":
            continue
        if allow and t not in allow:
            continue
        
        # Queue packet with simulated latency
        delay = BASE_LATENCY_S + random.uniform(-JITTER_S, JITTER_S)
        packet_queue.put(msg.get_msgbuf(), delay)
        
        counter[t] += 1
        now = time.time()
        if now - last_log >= PRINT_EVERY:
            last_log = now
            print(f"[mav_relay] {label}: queue={len(packet_queue._queue)} total={sum(counter.values())} last={t}")

def main():
    global stop_flag

    while True:
        try:
            udp_in  = make_conn(UDP_IN,  sysid=241, compid=191)
            udp_out = make_conn(UDP_OUT, sysid=241, compid=191)
            serial  = make_conn(SERIAL,  sysid=241, compid=191)

            print("[mav_relay] Links up. Starting pumps…")
            c1 = collections.Counter()
            c2 = collections.Counter()

            th1 = threading.Thread(target=pump, args=(udp_in, serial, ALLOW_GROUND_TO_AIR, "UDP->SER", c1), daemon=True)
            th2 = threading.Thread(target=pump, args=(serial, udp_out, ALLOW_AIR_TO_GROUND, "SER->UDP", c2), daemon=True)
            th1.start(); th2.start()

            # Liveness loop; if any link drops, reconnect
            while not stop_flag:
                for conn, name in ((udp_in,"udp_in"), (udp_out,"udp_out"), (serial,"serial")):
                    if conn and conn.portdead:
                        raise RuntimeError(f"{name} died")
                time.sleep(0.2)
        except KeyboardInterrupt:
            stop_flag = True
            break
        except Exception as e:
            print(f"[mav_relay] Link error: {e}; retrying in 1s…")
            time.sleep(1.0)
            continue

if __name__ == "__main__":
    def _sigint(_s,_f):
        print("\n[mav_relay] SIGINT -> shutting down")
        sys.exit(0)
    signal.signal(signal.SIGINT, _sigint)
    main()