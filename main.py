import numpy as np
import time
from numpy.lib.twodim_base import vander
import numpy.typing as npt

epsilon = 1.0E-5

def mag(x):
    return np.sqrt(x.dot(x))

def norm(x):
    m = mag(x)
    return x/(m if m != 0 else epsilon)

def anglerot(a, b):
    '''
    '''

    mag = angle(a, b)
    dir = norm(np.cross(a, b))
    return mag*dir

def angle(a, b):
    '''
    Computes the angle between two vectors in radians.
    Returns zero if the combined (multiplied) magnitude of a and b is less than the epsilon
    '''

    m = np.sqrt(a.dot(a)*b.dot(b))
    return 0 if m < epsilon else np.arccos(a.dot(b) / m)


def project(a, b):
    return np.dot(a, b) / mag(b)

class Navigation():
    def __init__(self, gain):
        self.gain = gain

    def process(self, r, vm, vt) -> npt.NDArray:
        return np.array([0])

def clamp_mag(x, max_m):
    xm = mag(x)
    if xm == 0:
        return x
    m = min(xm, max_m)
    return x/xm * m

def main():
    nav = Navigation(gain=4)

    rm = np.array([0, 0, 0], dtype=np.float64)
    rt = np.array([100, 100, 0], dtype=np.float64)

    vm = np.array([0, 0, 0], dtype=np.float64)
    vt = np.array([8, 0, 0], dtype=np.float64)
    prev_r = rt - rm

    print(f"mpos: {rm}")
    print(f"tpos: {rt}");

    thrust = 1
    turning_accel = 2

    ticks = 0

    while True:
        r = rt - rm

        if mag(r) < 5:
            print(f"reached target in {ticks} ticks")
            break

        er = r / mag(r)
        vr = np.dot(vm, er)

        #print(f"({100 if mag(vm) == 0 else vr / mag(vm) * 100}%) closing velocity: {vr}, velocity: {mag(vm)}")

        w = angle(r, prev_r)
        omega = norm(np.cross(prev_r, r))
        #omega = norm(np.cross(r, prev_r))

        #acceleration = nav.gain*w*vm
        #acceleration_dir = norm(np.cross(vm, omega))
        acceleration = np.cross(nav.gain * (w*omega), vm)

        print(f"dist: {mag(r)}, angle: {w}")

        vm += clamp_mag(acceleration, turning_accel)
        vm += norm(np.array([0, 1, 0] if mag(vm) == 0 else vm)) * thrust
        vm *= 0.95

        print(f"speed: {mag(vm)}")

        rm += vm
        rt += vt

        ticks += 1

        prev_r = r
        time.sleep(0.125)

if __name__ == '__main__':
    main()