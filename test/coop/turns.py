# SPDX-License-Identifier: MIT
"""Two agents, one per player, taking turns (threads here): the clock moves only once both are ready, by the fewer
ticks asked for, and the session ends where a director stepping the same inputs ends."""

import threading

ROUNDS = 4


def act(coop, player, car, round_):
    """What a player does at the start of each round: take its car, then drive and stop in turn"""
    if round_ == 0:
        player.look_at(car)
        player.press('V')
    elif round_ == 1:
        player.ask('key W down')
    elif round_ == 3:
        player.ask('key W up')


def run(coop):
    hashes = []
    for mode in ('director', 'turns'):
        s = coop.Session.launch(name='turns', scene='track', players=2)
        cars = [s.vehicle(0)['position'], s.vehicle(1)['position']]
        if mode == 'director':
            for r in range(ROUNDS):
                for k, p in enumerate(s.players):
                    act(coop, p, cars[k], r)
                s.step(20)
        else:
            replies = {}

            def play(k):
                p = coop.Player(k, s.players[k].port, s.players[k].pid, s.players[k].log)  # each agent its own connection
                for r in range(ROUNDS):
                    act(coop, p, cars[k], r)
                    # p1 asks for more, but the turn is the fewer asked for: 20 a round, as the director stepped
                    replies.setdefault(k, []).append(s.turn(k, 20 if k == 0 else 35))
                p.close()
            threads = [threading.Thread(target=play, args=(k,)) for k in range(2)]
            for t in threads:
                t.start()
            for t in threads:
                t.join(timeout=120)
            ticks = [[r['tick'] for r in replies.get(k, [])] for k in range(2)]
            coop.check(ticks[0] == ticks[1] == [20, 40, 60, 80], 'the turns should end on ticks 20, 40, 60, 80: %s' % ticks)
        coop.check(s.vehicle(0)['controller'] == 0 and s.vehicle(1)['controller'] == 1, 'each player should drive its car')
        hashes.append(s.host.state()['hash'])
        s.expect_sync()
        s.stop()
    coop.check(hashes[0] == hashes[1], 'turns and the director should end on one state: %s' % hashes)
