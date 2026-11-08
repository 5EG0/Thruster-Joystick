#!/usr/bin/env python3
"""
Kontrol Thruster dengan Joystick/Gamepad (ArduSub / ROV) via MAVLink.
Cara kerjanya sama seperti joystick di QGroundControl: stick -> MANUAL_CONTROL.

Instalasi:
    pip install pymavlink pygame

Menjalankan:
    python thruster_joystick.py                          # UDP 14550
    python thruster_joystick.py -c /dev/ttyACM0 -b py
    python thruster_joystick.py --show-axes              # cek nomor axis/tombol joystick

Pemetaan default (gamepad Xbox):
    Stick kiri  Y  : maju / mundur
    Stick kiri  X  : geser kiri / kanan (lateral)
    Stick kanan Y  : naik / turun (vertical)
    Stick kanan X  : yaw kiri / kanan

    Start : ARM            Back : DISARM
    X     : mode MANUAL    Y    : mode STABILIZE    D-pad atas : DEPTH_HOLD
    B     : STOP darurat (netral + disarm)
    LB / RB : kurangi / tambah gain (daya maksimum)

Jika axis tidak sesuai (beda OS/merek), jalankan --show-axes lalu atur
dengan --ax-fwd, --ax-lat, --ax-vert, --ax-yaw, dan --inv-* (lihat --help).

PERINGATAN: lepas propeller / pastikan thruster aman sebelum ARM.
"""

import argparse
import platform
import sys
import threading
import time

import pygame
from pymavlink import mavutil

# ----------------------------------------------------------------- konfigurasi
DEADZONE = 0.08       # abaikan gerakan stick kecil
EXPO = 0.35           # 0 = linear, 1 = sangat halus di tengah
LOOP_HZ = 30
GAIN_MIN, GAIN_MAX, GAIN_STEP = 0.1, 1.0, 0.1

# Nomor tombol (gamepad Xbox pada pygame 2)
BTN_A, BTN_B, BTN_X, BTN_Y = 0, 1, 2, 3
BTN_LB, BTN_RB, BTN_BACK, BTN_START = 4, 5, 6, 7


def default_axes():
    """Axis default: Linux (LX,LY,LT,RX,RY,RT) vs Windows/macOS (LX,LY,RX,RY,LT,RT)."""
    if platform.system() == "Linux":
        return dict(lat=0, fwd=1, yaw=3, vert=4)
    return dict(lat=0, fwd=1, yaw=2, vert=3)


def shape(v: float) -> float:
    """Deadzone + kurva expo, hasil -1..1."""
    if abs(v) < DEADZONE:
        return 0.0
    sign = 1.0 if v > 0 else -1.0
    v = (abs(v) - DEADZONE) / (1.0 - DEADZONE)      # skala ulang setelah deadzone
    v = (1 - EXPO) * v + EXPO * v ** 3                # expo
    return sign * min(1.0, v)


# -------------------------------------------------------------------- MAVLink
class Vehicle:
    def __init__(self, connection, baud):
        print(f"[+] Menghubungkan ke {connection} ...")
        self.m = mavutil.mavlink_connection(connection, baud=baud, source_system=255)
        print("[+] Menunggu heartbeat ...")
        self.m.wait_heartbeat()
        self.sysid = self.m.target_system
        self.compid = self.m.target_component
        self.armed = False
        self.mode = "?"
        self.battery = None
        self.depth = None
        self.heading = None
        self._run = True
        threading.Thread(target=self._hb_loop, daemon=True).start()
        print(f"[+] Terhubung: system={self.sysid}, component={self.compid}")

    def _hb_loop(self):
        while self._run:
            self.m.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            time.sleep(1)

    def poll(self):
        """Baca semua pesan masuk (non-blocking) dan perbarui status."""
        while True:
            msg = self.m.recv_match(blocking=False)
            if msg is None:
                break
            t = msg.get_type()
            if t == "HEARTBEAT" and msg.get_srcSystem() == self.sysid \
                    and msg.type != mavutil.mavlink.MAV_TYPE_GCS:
                self.armed = bool(
                    msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                rev = {v: k for k, v in self.m.mode_mapping().items()}
                self.mode = rev.get(msg.custom_mode, str(msg.custom_mode))
            elif t == "SYS_STATUS":
                self.battery = msg.voltage_battery / 1000.0
            elif t == "VFR_HUD":
                self.depth = msg.alt
                self.heading = msg.heading
            elif t == "STATUSTEXT":
                print("[FC]", msg.text)

    def arm(self, value: bool):
        self.m.mav.command_long_send(
            self.sysid, self.compid,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1 if value else 0, 0, 0, 0, 0, 0, 0)
        print("[+] ARM" if value else "[+] DISARM")

    def set_mode(self, name):
        mp = self.m.mode_mapping()
        if name not in mp:
            print(f"[!] Mode {name} tidak tersedia: {list(mp)}")
            return
        self.m.set_mode(mp[name])
        print(f"[+] Mode -> {name}")

    def send_manual(self, x, y, z, r, buttons=0):
        # x: maju(+), y: kanan(+), z: 0..1000 (500 netral, >500 naik), r: yaw kanan(+)
        self.m.mav.manual_control_send(self.sysid, x, y, z, r, buttons)

    def close(self):
        self._run = False


# ------------------------------------------------------------------------ GUI
class Display:
    def __init__(self):
        self.screen = pygame.display.set_mode((560, 330))
        pygame.display.set_caption("Kontrol Thruster - Joystick")
        self.font = pygame.font.SysFont("monospace", 18)
        self.big = pygame.font.SysFont("monospace", 26, bold=True)

    def bar(self, label, value, y):
        txt = self.font.render(f"{label:<9}{value:+5.2f}", True, (230, 230, 230))
        self.screen.blit(txt, (20, y))
        x0, w = 230, 300
        pygame.draw.rect(self.screen, (70, 70, 70), (x0, y + 4, w, 14))
        pygame.draw.line(self.screen, (200, 200, 200), (x0 + w // 2, y), (x0 + w // 2, y + 22))
        fill = int(value * w / 2)
        color = (80, 200, 120) if value >= 0 else (230, 140, 60)
        pygame.draw.rect(self.screen, color,
                         (x0 + w // 2 + min(0, fill), y + 4, abs(fill), 14))

    def draw(self, veh, cmd, gain, joy_name):
        self.screen.fill((25, 28, 35))
        state = "ARMED" if veh.armed else "DISARMED"
        col = (230, 70, 70) if veh.armed else (90, 200, 120)
        self.screen.blit(self.big.render(state, True, col), (20, 12))
        self.screen.blit(self.font.render(f"Mode: {veh.mode}", True, (230, 230, 230)), (230, 18))
        self.screen.blit(self.font.render(f"Gain: {int(gain * 100)}%", True, (230, 230, 230)), (400, 18))

        self.bar("Maju", cmd["fwd"], 70)
        self.bar("Lateral", cmd["lat"], 105)
        self.bar("Vertikal", cmd["vert"], 140)
        self.bar("Yaw", cmd["yaw"], 175)

        bat = f"{veh.battery:.2f} V" if veh.battery is not None else "-"
        dep = f"{veh.depth:.2f} m" if veh.depth is not None else "-"
        hdg = f"{veh.heading}°" if veh.heading is not None else "-"
        self.screen.blit(self.font.render(
            f"Baterai: {bat}   Kedalaman: {dep}   Heading: {hdg}", True, (180, 200, 255)), (20, 225))
        self.screen.blit(self.font.render(f"Joystick: {joy_name[:40]}", True, (150, 150, 150)), (20, 260))
        self.screen.blit(self.font.render(
            "Start=ARM Back=DISARM B=STOP X/Y=mode LB/RB=gain", True, (150, 150, 150)), (20, 290))
        pygame.display.flip()


# ----------------------------------------------------------------- show axes
def show_axes():
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("Tidak ada joystick terdeteksi.")
        return
    j = pygame.joystick.Joystick(0)
    j.init()
    print(f"Joystick: {j.get_name()}  axes={j.get_numaxes()} "
          f"buttons={j.get_numbuttons()} hats={j.get_numhats()}")
    print("Gerakkan stick / tekan tombol. Ctrl+C untuk keluar.\n")
    try:
        while True:
            pygame.event.pump()
            axes = " ".join(f"A{i}:{j.get_axis(i):+.2f}" for i in range(j.get_numaxes()))
            btns = "".join(str(j.get_button(i)) for i in range(j.get_numbuttons()))
            print(f"\r{axes} | btn {btns}   ", end="", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print()


# ----------------------------------------------------------------------- main
def main():
    d = default_axes()
    ap = argparse.ArgumentParser(description="Kontrol thruster dengan joystick via MAVLink")
    ap.add_argument("-c", "--connect", default="udpin:0.0.0.0:14550")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("-j", "--joy", type=int, default=0, help="indeks joystick")
    ap.add_argument("--gain", type=float, default=0.5, help="gain awal 0.1-1.0")
    ap.add_argument("--ax-fwd", type=int, default=d["fwd"])
    ap.add_argument("--ax-lat", type=int, default=d["lat"])
    ap.add_argument("--ax-vert", type=int, default=d["vert"])
    ap.add_argument("--ax-yaw", type=int, default=d["yaw"])
    ap.add_argument("--inv-fwd", action="store_true", help="balik arah maju/mundur")
    ap.add_argument("--inv-lat", action="store_true")
    ap.add_argument("--inv-vert", action="store_true")
    ap.add_argument("--inv-yaw", action="store_true")
    ap.add_argument("--show-axes", action="store_true", help="tampilkan nilai axis lalu keluar")
    args = ap.parse_args()

    if args.show_axes:
        show_axes()
        return

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() <= args.joy:
        print("[!] Joystick tidak ditemukan. Colokkan gamepad lalu coba lagi.")
        sys.exit(1)
    joy = pygame.joystick.Joystick(args.joy)
    joy.init()
    print(f"[+] Joystick: {joy.get_name()}")

    veh = Vehicle(args.connect, args.baud)
    disp = Display()
    clock = pygame.time.Clock()
    gain = max(GAIN_MIN, min(GAIN_MAX, args.gain))

    # Stick di-"nol"-kan dulu; sebagian driver melaporkan axis trigger = -1 saat awal
    cmd = dict(fwd=0.0, lat=0.0, vert=0.0, yaw=0.0)

    def axis(i, inv):
        try:
            v = joy.get_axis(i)
        except pygame.error:
            v = 0.0
        return -v if inv else v

    try:
        running = True
        while running:
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    running = False
                elif ev.type == pygame.JOYDEVICEREMOVED:
                    print("[!] Joystick terlepas! Thruster dinetralkan.")
                    veh.send_manual(0, 0, 500, 0)
                    veh.arm(False)
                    running = False
                elif ev.type == pygame.JOYBUTTONDOWN:
                    b = ev.button
                    if b == BTN_START:
                        veh.arm(True)
                    elif b == BTN_BACK:
                        veh.arm(False)
                    elif b == BTN_B:                      # STOP darurat
                        veh.send_manual(0, 0, 500, 0)
                        veh.arm(False)
                    elif b == BTN_X:
                        veh.set_mode("MANUAL")
                    elif b == BTN_Y:
                        veh.set_mode("STABILIZE")
                    elif b == BTN_RB:
                        gain = min(GAIN_MAX, gain + GAIN_STEP)
                    elif b == BTN_LB:
                        gain = max(GAIN_MIN, gain - GAIN_STEP)
                elif ev.type == pygame.JOYHATMOTION and ev.value == (0, 1):
                    veh.set_mode("DEPTH_HOLD")
                elif ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE:
                    running = False

            # Stick: pygame Y-axis negatif = ke atas, jadi dibalik agar atas = maju/naik
            cmd["fwd"] = shape(-axis(args.ax_fwd, args.inv_fwd))
            cmd["lat"] = shape(axis(args.ax_lat, args.inv_lat))
            cmd["vert"] = shape(-axis(args.ax_vert, args.inv_vert))
            cmd["yaw"] = shape(axis(args.ax_yaw, args.inv_yaw))

            x = int(cmd["fwd"] * gain * 1000)
            y = int(cmd["lat"] * gain * 1000)
            z = int(500 + cmd["vert"] * gain * 500)
            r = int(cmd["yaw"] * gain * 1000)
            veh.send_manual(x, y, z, r)

            veh.poll()
            disp.draw(veh, cmd, gain, joy.get_name())
            clock.tick(LOOP_HZ)
    except KeyboardInterrupt:
        pass
    finally:
        print("\n[+] Keluar: thruster dinetralkan.")
        for _ in range(5):                      # kirim netral beberapa kali
            veh.send_manual(0, 0, 500, 0)
            time.sleep(0.05)
        veh.close()
        pygame.quit()


if __name__ == "__main__":
    main()