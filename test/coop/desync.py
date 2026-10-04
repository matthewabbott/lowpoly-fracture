# SPDX-License-Identifier: MIT
"""A desync injected into p1's world (one body's velocity, one ulp) is found: the host stops the session, and both
windows report the same element and the tick it first differed."""


def run(coop):
    s = coop.Session.launch(name='desync', scene='track', players=2)
    p0, p1 = s.players
    s.step(200) # the track's debris is moving by then
    injected = p1.ask('inject')
    reply = p0.ask('step 60', ok=False)
    coop.check(reply.get('error') == 'stopped', 'the step should stop on the desync: %s' % reply)
    a, b = p0.state()['session'], p1.state()['session']
    coop.check(a['state'] == 'desync' and b['state'] == 'desync', 'both should be stopped by a desync: %s / %s' % (a, b))
    coop.check(a['report'] == b['report'], 'both should report the same: "%s" / "%s"' % (a['report'], b['report']))
    coop.check('first at tick %d' % injected['tick'] in a['report'] and 'bodies element' in a['report'],
               'the report should name the tick of the nudge (%d) and a body: %s' % (injected['tick'], a['report']))
    s.shot(ui=True, label='desync')
