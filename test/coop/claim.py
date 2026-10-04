# SPDX-License-Identifier: MIT
"""Two players on the track take cars with V: each its own, then both the same one on the same tick (the host's
player gets it, and the other's window must know it did not), then one leaves a car and the other takes it."""


def run(coop):
    s = coop.Session.launch(name='claim', scene='track', players=2)
    p0, p1 = s.players
    delay = p0.state()['session']['delay']

    # Each takes the car in front of its camera
    p0.look_at(s.vehicle(0)['position'])
    p1.look_at(s.vehicle(1)['position'])
    p0.press('V')
    p1.press('V')
    s.step(delay + 2)
    coop.check(s.vehicle(0)['controller'] == 0 and s.vehicle(1)['controller'] == 1,
               'each should drive its own car: controllers %d and %d' % (s.vehicle(0)['controller'], s.vehicle(1)['controller']))
    coop.check(p0.state()['driving'] == 0 and p1.state()['driving'] == 1, 'each window should say it drives its car')

    # Both drive off
    p0.ask('key W down')
    p1.ask('key W down')
    s.step(90)
    p0.ask('key W up')
    p1.ask('key W up')
    s.expect_sync()
    coop.check(s.vehicle(0)['speed'] > 3.0 and s.vehicle(1)['speed'] > 3.0,
               'both cars should be moving: %.1f and %.1f m/s' % (s.vehicle(0)['speed'], s.vehicle(1)['speed']))

    # Both get out, then both press V on car 2 in the same tick: commands apply in peer order, so p0 gets it
    p0.press('V')
    p1.press('V')
    s.step(delay + 2)
    car = s.vehicle(2)['position']
    p0.look_at(car)
    p1.look_at(car)
    p0.press('V')
    p1.press('V')
    s.step(delay + 2)
    coop.check(s.vehicle(2)['controller'] == 0, 'p0 should hold car 2, not %d' % s.vehicle(2)['controller'])
    coop.check(p1.state()['driving'] == -1, "p1's window should know it lost car 2 (it says it drives %d)" % p1.state()['driving'])
    s.shot(label='contest')

    # p0 gets out; p1 takes the car it left
    p0.press('V')
    s.step(delay + 2)
    p1.look_at(s.vehicle(2)['position'])
    p1.press('V')
    s.step(delay + 2)
    coop.check(s.vehicle(2)['controller'] == 1, 'p1 should take the car p0 left, but its controller is %d' % s.vehicle(2)['controller'])
    coop.check(p1.state()['driving'] == 2, 'p1 should drive car 2')
    s.expect_sync()
