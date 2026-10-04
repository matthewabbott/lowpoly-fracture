# SPDX-License-Identifier: MIT
"""p1's link is slow (100 ms and up to 40 more, each way): both players drive, held and then running at 60 Hz, and
the session stays in sync."""

import time


def run(coop):
    s = coop.Session.launch(name='latency', scene='track', players=2)
    p0, p1 = s.players
    p1.ask('net delay 100 40')
    for k, p in enumerate(s.players):
        p.look_at(s.vehicle(k)['position'])
        p.press('V')
    s.step(10)
    for p in s.players:
        p.ask('key W down')
    s.step(60)
    s.expect_sync()

    # Running: the host's clock at 60 Hz, as fast as the slow link lets it
    p0.ask('run')
    time.sleep(2.0)
    p0.ask('pause')
    closed = p0.state()['session']['closed']
    for p in s.players:
        p.ask('wait %d' % closed, timeout=30.0)
    tick = s.expect_sync()
    coop.check(tick > 90, 'the session should have run on (tick %d)' % tick)
    coop.check(s.vehicle(0)['controller'] == 0 and s.vehicle(1)['controller'] == 1, 'each should drive its car')
