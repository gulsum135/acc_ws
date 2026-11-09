#!/usr/bin/env python3
"""
ACC / AEB simulasyonu (kamerasiz, grafiksiz).

Tek dugum sanal bir dunyayi ve araci yonetir:
  - Arac duz bir yolda ilerler (x ekseni, metre).
  - Senaryo engelleri sirayla belirir ve kaybolur.
  - Kontrolcu mesafeye gore hizi ayarlar:
        SEYIR      -> yol bos, hedef hiza hizlan
        YAVASLIYOR -> engel yakin, rahat frenle durabilecegi hiza in
        ACIL_FREN  -> engel cok yakin, sert fren
        DURDU      -> hiz 0, engel hala orada
        HIZLANIYOR -> yol acildi / onundeki hizlandi, hedef hiza kademeli don
        TAKIP      -> onde yavas giden arac var, hizina uyup guvenli mesafede takip

Yayinlar:  /cmd_vel (Twist), /sim/obstacle_distance (Float32), /sim/state (String)
Sonuclar terminalde satir satir yazilir.
"""

import math
import os

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32, String

try:
    import cv2
    import numpy as np
except ImportError:  # OpenCV yoksa sadece terminal cikisi
    cv2 = None
    np = None

SEYIR = 'SEYIR'
YAVASLIYOR = 'YAVASLIYOR'
ACIL_FREN = 'ACIL_FREN'
DURDU = 'DURDU'
HIZLANIYOR = 'HIZLANIYOR'
TAKIP = 'TAKIP'


class World:
    """Sanal arac + senaryo engelleri (ROS'tan bagimsiz, test edilebilir)."""

    def __init__(self):
        self.t = 0.0
        self.car_x = 0.0
        self.speed = 0.0
        self.obstacle_x = None      # None = yol bos
        self.lead_speed = 0.0       # engelin hizi (m/s); 0 = duran engel
        self.lead_target = 0.0
        self.follow_timer = 0.0
        self.stopped_time = 0.0
        self.stage = 0
        self.stage_timer = 0.0
        self.finished = False
        self.min_gap = 999.0        # senaryo boyunca en kucuk mesafe (carpisma kontrolu)
        self.collided = False

    def update_scenario(self, dt):
        """Senaryo:
        1) 80 m'de duran bir arac belirir -> arac durur, 4 sn bekler, engel kalkar
        2) Arac hizlanir; x=150'de 15 m onde ani bir yaya belirir -> acil fren
           yaya 3 sn sonra gider, arac tekrar hizlanir
        3) x=200'de 45 m onde 4 m/s ile giden yavas bir arac belirir -> arac hizina
           uyar ve takip eder (TAKIP); 8 sn sonra onde giden hizlanir -> ego da hizlanir
        4) Onde giden gorus disina cikinca senaryo biter
        """
        if self.stage == 0:
            self.obstacle_x = 80.0
            self.stage = 1
        elif self.stage == 1:
            if self.speed < 0.05 and self.obstacle_x is not None:
                self.stage_timer += dt
                if self.stage_timer >= 4.0:
                    self.obstacle_x = None
                    self.stage = 2
        elif self.stage == 2:
            if self.car_x >= 150.0:
                self.obstacle_x = self.car_x + 15.0
                self.stage_timer = 0.0
                self.stage = 3
        elif self.stage == 3:
            self.stage_timer += dt
            if self.stage_timer >= 3.0:
                self.obstacle_x = None
                self.stage = 4
        elif self.stage == 4:
            # 3) x=200'de 45 m onde 4 m/s ile giden yavas bir arac belirir
            if self.car_x >= 200.0:
                self.obstacle_x = self.car_x + 45.0
                self.lead_speed = 4.0
                self.lead_target = 4.0
                self.follow_timer = 0.0
                self.stage = 5
        elif self.stage == 5:
            # Ego arac ona yetisip hizina uyunca 8 sn takip eder, sonra onde giden hizlanir
            d = self.distance()
            if d is not None and d < 25.0 and abs(self.speed - self.lead_speed) < 0.7:
                self.follow_timer += dt
            if self.follow_timer >= 8.0:
                self.lead_target = 14.0
                self.stage = 6
        elif self.stage == 6:
            d = self.distance()
            if d is not None and d > 70.0:     # sensor menzilinden cikti
                self.obstacle_x = None
                self.lead_speed = 0.0
                self.stage_timer = 0.0
                self.stage = 7
        elif self.stage == 7:
            self.stage_timer += dt
            if self.stage_timer >= 6.0:
                self.finished = True

        # Hareketli engelin kendi hizi (yumusak ivmelenme) ve konumu
        if self.obstacle_x is not None and self.lead_speed != self.lead_target:
            step = 1.0 * dt
            if self.lead_speed < self.lead_target:
                self.lead_speed = min(self.lead_target, self.lead_speed + step)
            else:
                self.lead_speed = max(self.lead_target, self.lead_speed - step)
        if self.obstacle_x is not None:
            self.obstacle_x += self.lead_speed * dt

    def distance(self):
        if self.obstacle_x is None:
            return None
        return self.obstacle_x - self.car_x


class Controller:
    """Mesafeye ve onundeki aracin hizina gore istenen hizi belirler."""

    def __init__(self, target_speed, stop_distance, critical_distance, comfort_decel,
                 accel, brake_decel, emergency_decel, headway=1.5):
        self.target = target_speed
        self.stop_d = stop_distance
        self.crit_d = critical_distance
        self.comfort = comfort_decel
        self.accel = accel
        self.brake = brake_decel
        self.emergency = emergency_decel
        self.headway = headway          # takip suresi (sn): hiz x headway kadar ek bosluk
        self.state = SEYIR
        self.resuming = False

    def step(self, speed, dist, dt, lead_speed=0.0):
        """(yeni_hiz, durum, gaz_yuzdesi, fren_yuzdesi) dondurur."""
        desired = self.target
        if dist is not None:
            gap_min = self.stop_d + 0.3
            # Rahat frenle onundeki aracin hizina inebilecegi azami hiz
            v_safe = math.sqrt(lead_speed ** 2 + 2.0 * self.comfort * max(0.0, dist - gap_min))
            # Takip: onundekinin hizi + mesafe hatasi (hedef mesafe hizla buyur)
            if lead_speed > 0.5:
                gap_target = gap_min + self.headway * speed
                v_follow = lead_speed + 0.6 * (dist - gap_target)
            else:
                v_follow = self.target      # duran engelde sadece rahat-fren egrisi gecerli
            desired = max(0.0, min(self.target, v_safe, v_follow))

            # Normal frenle yetisemiyorsa -> ACIL FREN
            closing = speed - lead_speed
            required = (closing * closing / (2.0 * max(0.1, dist - 1.0))) if closing > 0 else 0.0
            too_close = dist < self.crit_d and speed > v_safe + 0.5
            if speed > 0.1 and (required > self.brake or too_close):
                self.state = ACIL_FREN
                self.resuming = True
                new = max(0.0, speed - self.emergency * dt)
                return new, self.state, 0, 100

        if dist is not None and desired < self.target - 0.01:
            self.resuming = True
            if speed <= 0.05 and desired <= 0.05:
                self.state = DURDU
            elif speed > desired + 0.3:
                self.state = YAVASLIYOR
            elif speed < desired - 0.3:
                self.state = HIZLANIYOR
            elif lead_speed > 0.5:
                self.state = TAKIP
            else:
                self.state = YAVASLIYOR
        else:
            if self.resuming and speed < self.target - 0.05:
                self.state = HIZLANIYOR
            else:
                self.resuming = False
                self.state = SEYIR

        if speed > desired:
            new = max(desired, speed - self.brake * dt)
            brake_pct = int(min(100, 100 * (speed - new) / (self.brake * dt)))
            throttle_pct = 0
        elif speed < desired:
            new = min(desired, speed + self.accel * dt)
            throttle_pct = int(min(100, 100 * (new - speed) / (self.accel * dt)))
            brake_pct = 0
        else:
            new = speed
            throttle_pct = int(100 * speed / self.target) if self.target > 0 else 0
            brake_pct = 0
        return new, self.state, throttle_pct, brake_pct


def road_bar(dist, width=40, max_dist=60.0):
    """Terminal icin basit yol cizimi: A = arac, X = engel."""
    if dist is None:
        return 'A' + '.' * (width - 1)
    pos = int(min(max(dist, 0.0), max_dist) / max_dist * (width - 1))
    return 'A' + '.' * max(0, pos - 1) + 'X' + ' ' * max(0, width - 1 - pos)


STATE_COLORS = {  # BGR
    SEYIR: (60, 180, 60), YAVASLIYOR: (0, 150, 255), ACIL_FREN: (0, 0, 230),
    DURDU: (0, 0, 230), HIZLANIYOR: (200, 200, 0), TAKIP: (220, 160, 40),
}


def render_view(world, state, thr, brk, width=960, height=320):
    """Yukaridan yol gorunumu: arac sabit solda, dunya kayar."""
    img = np.full((height, width, 3), (45, 45, 45), np.uint8)
    ppm = 8.0                       # piksel / metre
    car_px = 160                    # aracin ekrandaki x konumu
    road_top, road_bot = 110, 230
    cv2.rectangle(img, (0, road_top), (width, road_bot), (70, 70, 70), -1)
    cv2.line(img, (0, road_top), (width, road_top), (200, 200, 200), 2)
    cv2.line(img, (0, road_bot), (width, road_bot), (200, 200, 200), 2)
    # Seritler (dunyaya sabit, arac ilerledikce akar)
    start = int(world.car_x // 6) * 6
    for wx in range(start - 12, start + int(width / ppm) + 12, 6):
        x = int(car_px + (wx - world.car_x) * ppm)
        cv2.line(img, (x, 170), (x + 24, 170), (220, 220, 220), 2)
    # Engel
    if world.obstacle_x is not None:
        ox = int(car_px + (world.obstacle_x - world.car_x) * ppm)
        if -40 < ox < width + 40:
            cv2.rectangle(img, (ox, 140), (ox + 60, 200), (0, 0, 220) if world.lead_speed < 0.1 else (170, 70, 170), -1)
            label = f'ARAC {world.lead_speed:.0f} m/s' if world.lead_speed > 0.1 else 'ENGEL'
            cv2.putText(img, label, (ox - 6, 132), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 255), 1, cv2.LINE_AA)
    # Arac
    col = STATE_COLORS.get(state, (200, 200, 200))
    cv2.rectangle(img, (car_px - 60, 140), (car_px, 200), col, -1)
    cv2.rectangle(img, (car_px - 60, 140), (car_px, 200), (255, 255, 255), 2)
    if brk > 0:
        cv2.rectangle(img, (car_px - 64, 146), (car_px - 58, 194), (0, 0, 255), -1)
    # Bilgi paneli
    dist = world.distance()
    cv2.rectangle(img, (0, 0), (width, 70), (25, 25, 25), -1)
    cv2.putText(img, f'Durum: {state}', (16, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2, cv2.LINE_AA)
    d_txt = f'{dist:.1f} m' if dist is not None else '--'
    cv2.putText(img, f'Hiz: {world.speed:.1f} m/s ({world.speed * 3.6:.0f} km/h)   Mesafe: {d_txt}',
                (16, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (230, 230, 230), 1, cv2.LINE_AA)
    # Gaz / Fren cubuklari
    for bx, val, c, name in ((width - 150, thr, (60, 180, 60), 'GAZ'), (width - 80, brk, (0, 0, 230), 'FREN')):
        cv2.rectangle(img, (bx, 250), (bx + 30, 310), (110, 110, 110), 1)
        h = int(60 * val / 100)
        cv2.rectangle(img, (bx, 310 - h), (bx + 30, 310), c, -1)
        cv2.putText(img, name, (bx - 2, 244), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1, cv2.LINE_AA)
    # Mesafe cubugu: durma / kritik / uyari bolgeleri icin alt serit
    cv2.putText(img, f'Yol: {world.car_x:.0f} m', (16, 304), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (200, 200, 200), 1, cv2.LINE_AA)
    return img


class AccSimNode(Node):
    def __init__(self):
        super().__init__('acc_sim_node')
        self.declare_parameter('target_speed_mps', 10.0)       # 36 km/h
        self.declare_parameter('stop_distance_m', 5.0)         # engelin onunde durulacak bosluk
        self.declare_parameter('critical_distance_m', 8.0)     # bunun altinda acil fren
        self.declare_parameter('comfort_decel', 2.0)           # m/s^2
        self.declare_parameter('acceleration', 1.5)            # m/s^2
        self.declare_parameter('braking', 3.0)                 # m/s^2
        self.declare_parameter('emergency_braking', 6.0)       # m/s^2
        self.declare_parameter('headway_sec', 1.5)             # takip suresi
        self.declare_parameter('step_hz', 20.0)
        self.declare_parameter('print_every_sec', 0.5)
        self.declare_parameter('show_window', True)

        g = lambda k: self.get_parameter(k).value
        self.ctrl = Controller(
            float(g('target_speed_mps')), float(g('stop_distance_m')),
            float(g('critical_distance_m')), float(g('comfort_decel')),
            float(g('acceleration')), float(g('braking')), float(g('emergency_braking')), float(g('headway_sec')))
        self.dt = 1.0 / max(1.0, float(g('step_hz')))
        self.print_every = float(g('print_every_sec'))
        sw = g('show_window')
        want_window = (sw.lower() in ('true', '1')) if isinstance(sw, str) else bool(sw)
        self.show_window = bool(want_window and cv2 is not None and os.environ.get('DISPLAY'))
        if want_window and not self.show_window:
            print('(Pencere acilamadi: OpenCV ya da DISPLAY yok, sadece terminal cikisi.)')
        self.world = World()
        self.last_print = -999.0

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_dist = self.create_publisher(Float32, '/sim/obstacle_distance', 10)
        self.pub_state = self.create_publisher(String, '/sim/state', 10)
        self.timer = self.create_timer(self.dt, self.tick)

        self.get_logger().info('ACC/AEB simulasyonu basladi. Cikmak icin Ctrl+C.')
        print(f"{'sure':>6} {'durum':<11} {'hiz m/s':>8} {'km/h':>6} {'mesafe':>8} "
              f"{'engel v':>8} {'gaz':>4} {'fren':>5}  yol (A=arac, X=engel)")

    def tick(self):
        w = self.world
        w.update_scenario(self.dt)
        dist = w.distance()
        w.speed, state, thr, brk = self.ctrl.step(w.speed, dist, self.dt, w.lead_speed)
        w.car_x += w.speed * self.dt
        w.t += self.dt
        dist_after = w.distance()
        if dist_after is not None:
            w.min_gap = min(w.min_gap, dist_after)
            if dist_after <= 0.0:
                w.collided = True

        twist = Twist()
        twist.linear.x = float(w.speed)
        self.pub_cmd.publish(twist)
        self.pub_dist.publish(Float32(data=float(dist if dist is not None else 999.0)))
        self.pub_state.publish(String(data=state))

        if w.t - self.last_print >= self.print_every or w.finished:
            d_txt = f'{dist:6.1f} m' if dist is not None else '     --'
            print(f'{w.t:6.1f} {state:<11} {w.speed:8.2f} {w.speed * 3.6:6.1f} {d_txt:>8} '
                  f'{w.lead_speed:8.1f} {thr:3d}% {brk:4d}%  {road_bar(dist)}')
            self.last_print = w.t

        if self.show_window:
            cv2.imshow('ACC/AEB Simulasyonu', render_view(w, state, thr, brk))
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q')):
                self.report()
                raise SystemExit

        if w.finished:
            self.report()
            raise SystemExit

    def report(self):
        w = self.world
        print('\n=== SENARYO BITTI ===')
        print(f'Toplam sure: {w.t:.1f} s, katedilen yol: {w.car_x:.0f} m')
        print(f'Engele en yakin mesafe: {w.min_gap:.1f} m')
        print('Carpisma: ' + ('EVET (basarisiz)' if w.collided else 'YOK (basarili)'))


def main(args=None):
    rclpy.init(args=args)
    node = AccSimNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if node.show_window:
            cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
