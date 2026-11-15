"""Pure time-based foot trajectory primitives."""
import math

LEGS = ('fl', 'fr', 'bl', 'br')
CRAWL_OFFSETS = {'fl': .75, 'br': .50, 'fr': .25, 'bl': 0.0}
TROT_OFFSETS = {'fl': 0.0, 'br': 0.0, 'fr': .5, 'bl': .5}


def smooth(value):
    value = max(0., min(1., float(value)))
    return value * value * (3. - 2. * value)


def foot_trajectory(leg, time_s, command, step_length, lift_height, cycle_duration,
                    gait='crawl', swing_fraction=.125):
    """Return (forward, lateral, lift) offsets; one swing per cycle."""
    offsets = CRAWL_OFFSETS if gait == 'crawl' else TROT_OFFSETS
    phase = (float(time_s) / cycle_duration + offsets[leg]) % 1.
    stance = 1. - swing_fraction
    if phase >= stance:
        progress = (phase - stance) / swing_fraction
        wave = -.5 + smooth(progress)
        lift = lift_height * math.sin(math.pi * progress) ** 2
    else:
        wave = .5 - phase / stance
        lift = 0.
    return (wave * step_length * float(command[0]),
            wave * step_length * float(command[1]), lift)


def swing_legs(time_s, cycle_duration, gait='crawl', swing_fraction=.125):
    offsets = CRAWL_OFFSETS if gait == 'crawl' else TROT_OFFSETS
    phase = (time_s / cycle_duration) % 1.
    return tuple(leg for leg in LEGS
                 if (phase + offsets[leg]) % 1. >= 1. - swing_fraction)
