# SPDX-License-Identifier: MIT
"""The same requests in the same order give the same session: two runs of a drive with a grenade thrown give equal
hashes after every step (the camera, the aim and the held keys all move with the ticks, not the frames)."""


def play(coop):
    s = coop.Session.launch(name='reproducible', scene='track', players=2)
    p0, p1 = s.players
    hashes = []
    p0.look_at(s.vehicle(0)['position'])
    p0.press('V')
    hashes.append(s.step(10)['hash'])
    p0.ask('key W down')
    p0.ask('key A down')  # steering into a turn: the chase camera swings with the car
    hashes.append(s.step(45)['hash'])
    p0.ask('key A up')
    # p1 throws a grenade from its camera at the car p0 drives, wherever the car got to
    p1.look_at(s.vehicle(0)['position'], back=6.0, up=3.0)
    p1.press('2')
    p1.ask('mouse down')
    p1.ask('mouse up')
    hashes.append(s.step(60)['hash'])
    p0.ask('key W up')
    hashes.append(s.step(30)['hash'])
    s.expect_sync()
    s.stop()
    return hashes


def run(coop):
    a = play(coop)
    b = play(coop)
    coop.check(a == b, 'two runs should agree step for step:\n  %s\n  %s' % (a, b))
