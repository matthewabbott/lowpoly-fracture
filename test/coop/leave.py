# SPDX-License-Identifier: MIT
"""Three players; p2 quits: the host and p1 both stop, and both say who left (nobody waits for ever)."""

import time


def run(coop):
    s = coop.Session.launch(name='leave', scene='track', players=3)
    p0, p1, p2 = s.players
    s.step(30)
    s.expect_sync()
    p2.ask('quit', ok=False)
    reply = p0.ask('step 30', ok=False)
    coop.check(reply.get('error') == 'stopped' and reply.get('report') == 'peer 2 left',
               'the host should stop because peer 2 left: %s' % reply)
    for _ in range(100):
        if p1.state()['session']['state'] == 'stopped':
            break
        time.sleep(0.05)
    b = p1.state()['session']
    coop.check(b['state'] == 'stopped' and b['report'] == 'peer 2 left', 'p1 should be told peer 2 left: %s' % b)
