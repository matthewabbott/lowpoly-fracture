# SPDX-License-Identifier: MIT
"""A third sandbox joining a session that has started is refused, and told it came late (no catch-up yet)."""

import os
import time


def run(coop):
    s = coop.Session.launch(name='late', scene='track', players=2)
    s.step(30)
    log = os.path.join(s.dir, 'late.log')
    process = coop._start(['--scene', 'track', '--control', '0', '--background', '--width', '640', '--height', '360',
                           '--join', '127.0.0.1:%d' % s.game], log)
    try:
        port = int(coop._wait_for(log, r'control 127\.0\.0\.1:(\d+)', process).group(1))
        late = coop.Player(9, port, process.pid, log)
        for _ in range(100):
            session = late.state()['session']
            if session['state'] != 'joining':
                break
            time.sleep(0.05)
        coop.check(session['state'] == 'refused' and 'late' in session['report'], 'it should be refused as late: %s' % session)
        late.ask('quit', ok=False)
        late.close()
        s.step(10)  # the others play on
        s.expect_sync()
    finally:
        if coop._alive(process.pid):
            coop._kill(process.pid)
