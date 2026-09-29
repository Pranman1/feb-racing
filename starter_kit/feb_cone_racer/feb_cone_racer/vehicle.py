"""The car, as measured in the simulator (tools/dynid.py records it, tools/dynfit.py checks it).

Everything the racing half needs to know about the vehicle is here, in one place, with where
each number came from. The measurements, in short:

  * The throttle sets the WHEEL speed, at once: 25.1 m/s per unit of throttle. The motor is far
    stronger than the tyres, so the wheels are at their target within a physics step.
  * The body follows the wheels through the tyre. The force depends on the slip ratio
    (wheel - body) / body: it rises 50 m/s^2 per unit of slip, peaks at 7.2 m/s^2 near 15%
    slip, and falls to 5.4 m/s^2 once the tyre slides (30% and beyond). The same curve brakes
    the car when the wheels turn slower than the body. Throttle exactly zero locks the wheels,
    which is simply the sliding end of that curve; it is not a stronger brake than lifting.
  * So the wheel encoders measure the wheel speed, which is the command coming back, and only
    agree with the body when the tyre is not slipping much.
  * Sideways the car is a kinematic bicycle to within 4% (the rear slip angle at 6.2 m/s^2 is
    0.2 degrees) until the tyres let go at about 6.3 m/s^2; past that it ploughs at about 5.
  * The steering servo slews at 3.2 rad/s, and both commands take effect about 0.12 s after
    they are sent (one bridge tick plus transport).
"""
import math

import numpy as np

WHEEL_RADIUS = 0.059        # m, the wheel colliders
WHEELBASE = 0.339           # m, effective: yaw rate = v tan(delta) / L fits every steering hold to 1%
AXLE_TO_CENTRE = 0.155      # m from the rear axle to the centre of mass (body velocities are measured there)
AXLE_TO_LIDAR = 0.2733      # m from the rear axle to the lidar (the pose the map match estimates)
MAX_STEER = 0.5236          # rad at the wheels for a steering command of 1
STEER_RATE = 3.2            # rad/s, the servo's slew rate
ACT_DELAY = 0.12            # s from publishing a command to the simulator applying it
WHEEL_SPEED_PER_THROTTLE = 25.11   # m/s of wheel speed per unit throttle
DRAG = 0.273                # 1/s, the body's linear drag (deceleration = DRAG * v)
SLIP_STIFFNESS = 50.0       # m/s^2 per unit slip ratio, small slip
SLIP_PEAK = 0.145           # slip ratio of the peak
MU_PEAK = 7.2               # m/s^2 at the peak
SLIP_SLIDE = 0.30           # slip ratio beyond which the tyre slides
MU_SLIDE = 5.38             # m/s^2 sliding
LAT_PEAK = 6.3              # m/s^2 of cornering before the tyres let go
LAT_SLIDE = 5.0             # m/s^2 of cornering once they have
CORNER_DRAG = 1.0           # deceleration = CORNER_DRAG * |lateral acceleration * steering angle|
V_SLIP_MIN = 0.4            # m/s, floor of the slip ratio's denominator


def mu_long(slip):
    """Tyre force per unit mass (m/s^2) along the car for a slip ratio; odd in the slip."""
    s = abs(slip)
    if s <= SLIP_PEAK:
        f = MU_PEAK * s / SLIP_PEAK
    elif s <= SLIP_SLIDE:
        f = MU_PEAK + (MU_SLIDE - MU_PEAK) * (s - SLIP_PEAK) / (SLIP_SLIDE - SLIP_PEAK)
    else:
        f = MU_SLIDE
    return math.copysign(f, slip)


def slip_for(accel):
    """The slip ratio that gives a tyre force (m/s^2), on the rising side of the curve."""
    a = float(np.clip(accel, -MU_PEAK, MU_PEAK))
    return a * SLIP_PEAK / MU_PEAK


def throttle_for(v_body, accel):
    """The throttle that makes the tyre push the body at `accel` (m/s^2, drag not included) when
    the body moves at v_body: the wheel speed that puts the tyre at the right slip."""
    wheel = max(v_body, 0.0) * (1.0 + slip_for(accel))
    if v_body < V_SLIP_MIN:                      # from rest the ratio means nothing: ask for the speed a moment ahead
        wheel = max(wheel, v_body + 0.06 * max(accel, 0.0))
    return wheel / WHEEL_SPEED_PER_THROTTLE


class BodySpeed:
    """The body's speed from the wheel encoders: the wheel speed put through the tyre.

    The encoders count the wheels, and under power or braking the wheels are not the car. The
    tyre curve says how hard a given slip pushes the body, so integrating it from the measured
    wheel speed gives the body's speed, without the accelerometer and without the map."""

    def __init__(self):
        self.v = 0.0

    def update(self, wheel_speed, dt, yaw_rate=0.0, steer=0.0):
        n = max(int(math.ceil(dt / 0.005)), 1)
        h = dt / n
        for _ in range(n):
            slip = (wheel_speed - self.v) / max(abs(self.v), V_SLIP_MIN)
            a = mu_long(slip) - DRAG * self.v - CORNER_DRAG * abs(self.v * yaw_rate * steer)
            v_new = self.v + a * h
            # the tyre pulls the body towards the wheel speed and never through it
            if (self.v - wheel_speed) * (v_new - wheel_speed) < 0.0:
                v_new = wheel_speed
            self.v = v_new
        return self.v


class Plant:
    """The car for the offline bench: the measured physics, integrated at 1 ms. State is the
    rear axle's pose, the body speed, the steering angle and the wheel angle the encoders
    count; commands reach it after the actuation delay."""

    def __init__(self, x=0.0, y=0.0, psi=0.0, v=0.0):
        self.x, self.y, self.psi, self.v = x, y, psi, v
        self.delta = 0.0
        self.wheel_angle = 0.0
        self.yaw_rate = 0.0
        self.t = 0.0
        self.queue = []                  # (time it takes effect, steer command rad, throttle)
        self.steer_cmd, self.throttle = 0.0, 0.0
        self.sliding = 0.0               # seconds spent past the cornering limit

    def command(self, steer_rad, throttle, delay=ACT_DELAY):
        self.queue.append((self.t + delay, float(np.clip(steer_rad, -MAX_STEER, MAX_STEER)), float(np.clip(throttle, -1.0, 1.0))))

    def step(self, dt, h=0.001):
        for _ in range(max(int(round(dt / h)), 1)):
            while self.queue and self.queue[0][0] <= self.t + 1e-9:
                _, self.steer_cmd, self.throttle = self.queue.pop(0)
            self.delta += float(np.clip(self.steer_cmd - self.delta, -STEER_RATE * h, STEER_RATE * h))
            wheel = WHEEL_SPEED_PER_THROTTLE * self.throttle
            slip = (wheel - self.v) / max(abs(self.v), V_SLIP_MIN)
            r_kin = self.v * math.tan(self.delta) / WHEELBASE
            ay = self.v * r_kin
            if abs(ay) > LAT_PEAK:       # past the limit the car ploughs: the tyres give what they give sliding
                r = math.copysign(LAT_SLIDE / max(abs(self.v), 0.3), r_kin)
                self.sliding += h
            else:
                r = r_kin
            a = mu_long(slip) - DRAG * self.v - CORNER_DRAG * abs(self.v * r * self.delta)
            v_new = self.v + a * h
            if (self.v - wheel) * (v_new - wheel) < 0.0:
                v_new = wheel
            self.v = v_new
            self.x += self.v * math.cos(self.psi) * h
            self.y += self.v * math.sin(self.psi) * h
            self.psi += r * h
            self.yaw_rate = r
            self.wheel_angle += wheel / WHEEL_RADIUS * h
            self.t += h

    def point(self, ahead):
        """A point of the car `ahead` of the rear axle."""
        return np.array([self.x + ahead * math.cos(self.psi), self.y + ahead * math.sin(self.psi)])
