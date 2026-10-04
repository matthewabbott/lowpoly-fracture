# SPDX-License-Identifier: MIT
"""A third sandbox joining a session that has started is refused, and told it came late (no catch-up yet)."""

import time


def run(coop):
    s = coop.Session.launch(name='late', scene='track', players=2)
    s.step(30)
    late = s.join_late()
    for _ in range(100):
        session = late.state()['session']
        if session['state'] != 'joining':
            break
        time.sleep(0.05)
    coop.check(session['state'] == 'refused' and 'late' in session['report'], 'it should be refused as late: %s' % session)
    s.step(10)  # the others play on
    s.expect_sync()
