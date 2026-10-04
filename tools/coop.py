#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""coop.py: agent-driven co-op sessions of the sandbox (one window per player), for testing multiplayer by hand or
by agent. Python 3, standard library only. Each sandbox listens on a loopback control port (app/sandbox/control.h);
this launches them, keeps where they are in build/coop/<name>/session.json, and sends them requests. Every command
prints the JSON replies, one line per player.

  python tools/coop.py launch [--scene track] [--players 2] [--delay 4] [--size 960x540] [--name NAME]
                              [--camera p1=x,y,z,yaw,pitch] [--script p1=file] [--bombard N] [--running] [--allow-input]
  python tools/coop.py p1 key V tap          any request to one player (p0 is the host), or to "all" of them
  python tools/coop.py all state
  python tools/coop.py step 60               the session's clock: every machine steps 60 ticks, then holds
  python tools/coop.py turn p1 30            p1 is ready for 30 more ticks; returns once every player has asked
  python tools/coop.py shot [p1|all] [--ui]  PNGs in build/coop/NAME/shots/ (all: one side by side, too)
  python tools/coop.py sync                  every player at one tick with one hash? (exit 1 if not)
  python tools/coop.py net p1 delay 120 40   p1's link held 120 ms and up to 40 more, each way (stall MS: a pause)
  python tools/coop.py log p1 [lines]        the end of a player's log (its reports, a desync's)
  python tools/coop.py stop                  quit every window of the session
  python tools/coop.py test [name ...]       the scenarios in test/coop/*.py, each in its own session
--name NAME picks the session on any command (default: "default"); several sessions may run at once.

Requests a player takes (control.h has them all):
  state | key A-Z|0-9|SPACE|SHIFT|F1|F12 down|up|tap | mouse down|up | camera x y z [yaw pitch | at x y z]
  cmd <script line without its tick> | pause | run | step N | wait T | turn PEER N | shot PATH [ui] | dump PATH
  inject | net delay MS [JITTER] | net stall MS | quit

How the sandbox plays (tips for agents):
- The clock is held: nothing moves until step (from the host) or every player's turn. The same requests in the same
  order give the same session, hash for hash.
- A key acts as a person's would: V takes the nearest free car (or mech) within 25 m of the camera, or leaves the one
  driven; W/S/A/D drive while held (key W down, step, key W up); 1-7 pick a tool; the left button fires it at the
  screen's centre. Point the camera first: camera x y z at x y z. While driving, the camera chases the car.
- A player's input applies after the session's input delay (state's session.delay, 4 by default): step at least
  delay + 1 ticks before looking for what it did. state's vehicles list each car's controller (-1: the scene's
  driver, else the peer driving it) and position.
- A screenshot is a PNG: read it to see. --ui adds the sandbox's panel (the co-op line, a desync's report).
"""

import argparse
import importlib.util
import json
import os
import re
import socket
import struct
import subprocess
import sys
import time
import zlib

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SANDBOX = os.environ.get('LPF_SANDBOX', os.path.join(REPO, 'build', 'msvc-release', 'bin',
                                                      'sandbox.exe' if os.name == 'nt' else 'sandbox'))
ROOT = os.path.join(REPO, 'build', 'coop')


class CoopError(Exception):
    pass


def check(condition, message):
    """A scenario's assertion: fails it with the message"""
    if not condition:
        raise AssertionError(message)


# ---- one sandbox ----

class Player:
    def __init__(self, peer, port, pid, log):
        self.peer, self.port, self.pid, self.log = peer, port, pid, log
        self.name = 'p%d' % peer
        self._socket = None
        self._buffer = b''

    def ask(self, line, timeout=30.0, ok=True):
        """Sends one request and returns its reply (a dict); raises CoopError on a refusal unless ok=False"""
        try:
            if self._socket is None:
                self._socket = socket.create_connection(('127.0.0.1', self.port), timeout=10.0)
            self._socket.settimeout(timeout)
            self._socket.sendall((line + '\n').encode())
            while b'\n' not in self._buffer:
                chunk = self._socket.recv(1 << 16)
                if not chunk:
                    raise CoopError('%s closed the connection (did it quit?)' % self.name)
                self._buffer += chunk
        except (socket.timeout, OSError) as e:
            self.close()
            raise CoopError('%s: no answer to "%s" (%s)' % (self.name, line, e))
        reply, self._buffer = self._buffer.split(b'\n', 1)
        result = json.loads(reply)
        if ok and not result.get('ok'):
            raise CoopError('%s refused "%s": %s' % (self.name, line, result.get('report') or result.get('error')))
        return result

    def state(self):
        return self.ask('state')

    def look_at(self, point, back=8.0, up=5.0):
        """The free camera behind and above a point, looking at it: V takes the car nearest the camera"""
        x, y, z = point
        return self.ask('camera %.3f %.3f %.3f at %.3f %.3f %.3f' % (x, y + up, z + back, x, y, z))

    def press(self, key):
        return self.ask('key %s tap' % key)

    def close(self):
        if self._socket is not None:
            self._socket.close()
        self._socket = None
        self._buffer = b''

    def tail(self, lines=20):
        try:
            with open(self.log, encoding='utf-8', errors='replace') as f:
                return ''.join(f.readlines()[-lines:])
        except OSError:
            return ''


def _wait_for(log, pattern, process, seconds=30.0):
    """The first match of pattern in a log a process is writing"""
    end = time.time() + seconds
    while time.time() < end:
        try:
            with open(log, encoding='utf-8', errors='replace') as f:
                m = re.search(pattern, f.read())
            if m:
                return m
        except OSError:
            pass
        if process.poll() is not None:
            raise CoopError('it exited (code %s) before printing "%s"; its log: %s' % (process.returncode, pattern, log))
        time.sleep(0.05)
    raise CoopError('nothing like "%s" in %s within %d s' % (pattern, log, seconds))


def _start(args, log):
    flags = 0
    if os.name == 'nt':
        flags = subprocess.CREATE_NO_WINDOW | subprocess.CREATE_NEW_PROCESS_GROUP
    with open(log, 'w') as out:
        # Detached: no pipes of ours, so the shell that ran coop.py returns while the windows stay
        return subprocess.Popen([SANDBOX] + args, cwd=REPO, stdin=subprocess.DEVNULL, stdout=out,
                                stderr=subprocess.STDOUT, close_fds=True, creationflags=flags)


def _screen_width():
    try:
        import ctypes
        return ctypes.windll.user32.GetSystemMetrics(0)
    except Exception:
        return 1920


# ---- a session ----

class Session:
    def __init__(self, name, players, scene, game=None):
        self.name, self.players, self.scene, self.game = name, players, scene, game
        self.dir = os.path.join(ROOT, name)

    @property
    def host(self):
        return self.players[0]

    def player(self, which):
        """p0..pN, or the host"""
        if which == 'host':
            return self.host
        m = re.fullmatch(r'p(\d+)', which)
        if not m or int(m.group(1)) >= len(self.players):
            raise CoopError('no player "%s" (this session has p0..p%d)' % (which, len(self.players) - 1))
        return self.players[int(m.group(1))]

    @staticmethod
    def launch(name='default', scene='track', players=2, delay=4, size=(960, 540), cameras=None, scripts=None,
               bombard=0, running=False, allow_input=False):
        """Starts one sandbox per player (p0 hosts), held, and returns once every one is in the session"""
        Session.stop_named(name)
        folder = os.path.join(ROOT, name)
        os.makedirs(os.path.join(folder, 'shots'), exist_ok=True)
        if not os.path.exists(SANDBOX):
            raise CoopError('no sandbox at %s (pwsh tools/build.ps1)' % SANDBOX)
        cameras, scripts = cameras or {}, scripts or {}
        columns = max(1, min(players, _screen_width() // (size[0] + 16)))
        common = ['--scene', scene, '--control', '0', '--background', '--width', str(size[0]), '--height',
                  str(size[1]), '--input-delay', str(delay)]
        common += [] if running else ['--paused']
        common += ['--allow-input'] if allow_input else []
        common += ['--bombard', str(bombard)] if bombard > 0 else []
        session = Session(name, [], scene)
        game = None
        for k in range(players):
            args = list(common)
            args += ['--window', '%d,%d' % ((k % columns) * (size[0] + 16), (k // columns) * (size[1] + 40))]
            if 'p%d' % k in cameras:
                args += ['--camera', cameras['p%d' % k]]
            if 'p%d' % k in scripts:
                args += ['--script', scripts['p%d' % k]]
            if players > 1:
                args += ['--host', '127.0.0.1:0', '--peers', str(players - 1)] if k == 0 else ['--join', '127.0.0.1:%d' % game]
            log = os.path.join(folder, 'p%d.log' % k)
            process = _start(args, log)
            port = int(_wait_for(log, r'control 127\.0\.0\.1:(\d+)', process).group(1))
            if k == 0 and players > 1:
                game = int(_wait_for(log, r'co-op: hosting \S+ on port (\d+)', process).group(1))
            player = Player(k, port, process.pid, log)
            session.players.append(player)
            if k > 0:
                # Welcome before the next joins, so peer ids follow the order of launch
                end = time.time() + 30.0
                while player.state().get('peer') != k:
                    if time.time() > end:
                        raise CoopError('p%d was not welcome as peer %d: %s' % (k, k, player.tail(5)))
                    time.sleep(0.05)
        session.game = game
        session.save()
        if players > 1:
            session.wait_running()
        return session

    def wait_running(self, seconds=30.0):
        end = time.time() + seconds
        while True:
            states = [p.state() for p in self.players]
            if all(s['session']['state'] == 'running' for s in states):
                return states
            if any(s['session']['state'] not in ('joining', 'running') for s in states) or time.time() > end:
                raise CoopError('the session did not start: %s' % [s['session'] for s in states])
            time.sleep(0.05)

    def save(self):
        data = {'name': self.name, 'scene': self.scene, 'game': self.game,
                'players': [{'peer': p.peer, 'port': p.port, 'pid': p.pid, 'log': p.log} for p in self.players]}
        with open(os.path.join(self.dir, 'session.json'), 'w') as f:
            json.dump(data, f, indent=1)

    @staticmethod
    def load(name='default'):
        path = os.path.join(ROOT, name, 'session.json')
        if not os.path.exists(path):
            raise CoopError('no session "%s" (coop.py launch --name %s)' % (name, name))
        with open(path) as f:
            data = json.load(f)
        return Session(data['name'], [Player(p['peer'], p['port'], p['pid'], p['log']) for p in data['players']],
                       data['scene'], data.get('game'))

    # -- the clock --

    def step(self, ticks):
        """Every machine steps `ticks` more, then holds; returns the host's reply (tick, hash)"""
        return self.host.ask('step %d' % ticks, timeout=30.0 + 0.05 * ticks)

    def turn(self, peer, ticks, timeout=600.0):
        """Player `peer` is ready for `ticks` more; returns once every player has asked, and the clock has moved"""
        player = Player(0, self.host.port, self.host.pid, self.host.log)  # its own connection: turns wait together
        try:
            return player.ask('turn %d %d' % (peer, ticks), timeout=timeout)
        finally:
            player.close()

    # -- looking --

    def states(self):
        return [p.state() for p in self.players]

    def vehicle(self, index):
        """A vehicle as the host sees it: controller (-1: the scene's driver), position, speed, hud"""
        return self.host.state()['vehicles'][index]

    def expect_sync(self):
        """Every player at one tick with one hash (after a step, all are); returns that tick"""
        states = self.states()
        ticks = {s['tick'] for s in states}
        hashes = {s['hash'] for s in states}
        check(len(ticks) == 1 and len(hashes) == 1,
              'out of step: %s' % ', '.join('p%d tick %d hash %s' % (i, s['tick'], s['hash']) for i, s in enumerate(states)))
        return ticks.pop()

    def shot(self, which=None, ui=False, label=None):
        """Screenshots of the players (all by default): their paths, and for several a side-by-side composite last"""
        players = self.players if which in (None, 'all') else [self.player(which)]
        tick = self.host.state()['tick']
        stem = label or 't%05d' % tick
        paths = []
        for p in players:
            path = os.path.join(self.dir, 'shots', '%s_%s%s.png' % (stem, p.name, '_ui' if ui else ''))
            p.ask('shot %s%s' % (path, ' ui' if ui else ''))
            paths.append(path)
        if len(paths) > 1:
            composite = os.path.join(self.dir, 'shots', '%s_all%s.png' % (stem, '_ui' if ui else ''))
            side_by_side(paths, composite)
            paths.append(composite)
        return paths

    # -- the end --

    def stop(self):
        alive = [p for p in self.players if _alive(p.pid)]
        for p in alive:
            try:
                p.ask('quit', timeout=5.0, ok=False)
            except CoopError:
                pass
            p.close()
        end = time.time() + 5.0
        while any(_alive(p.pid) for p in alive) and time.time() < end:
            time.sleep(0.1)
        for p in alive:
            if _alive(p.pid):
                _kill(p.pid)

    @staticmethod
    def stop_named(name):
        try:
            Session.load(name).stop()
        except (CoopError, OSError, ValueError, KeyError):
            pass


def _alive(pid):
    """Whether that pid is still a sandbox"""
    if os.name == 'nt':
        out = subprocess.run(['tasklist', '/FI', 'PID eq %d' % pid, '/NH'], capture_output=True, text=True).stdout
        return 'sandbox' in out.lower()
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def _kill(pid):
    """Ends a sandbox that did not quit when asked"""
    if os.name == 'nt':
        subprocess.run(['taskkill', '/PID', str(pid), '/F'], capture_output=True)
    else:
        try:
            os.kill(pid, 9)
        except OSError:
            pass


# ---- PNGs (the sandbox writes RGB8 with filter 0; the composite is compressed) ----

def read_png(path):
    with open(path, 'rb') as f:
        data = f.read()
    if data[:8] != b'\x89PNG\r\n\x1a\n':
        raise CoopError('%s is not a PNG' % path)
    at, idat, width, height = 8, b'', 0, 0
    while at < len(data):
        length, kind = struct.unpack('>I4s', data[at:at + 8])
        body = data[at + 8:at + 8 + length]
        if kind == b'IHDR':
            width, height, depth, colour = struct.unpack('>IIBB', body[:10])
            if depth != 8 or colour != 2:
                raise CoopError('%s: only 8-bit RGB is read here' % path)
        elif kind == b'IDAT':
            idat += body
        at += 12 + length
    raw = zlib.decompress(idat)
    stride = 3 * width
    rows = []
    for y in range(height):
        row = raw[y * (stride + 1):(y + 1) * (stride + 1)]
        if row[0] != 0:
            raise CoopError('%s: a filtered row (only filter 0 is read here)' % path)
        rows.append(row[1:])
    return width, height, rows


def write_png(path, width, height, rows):
    raw = b''.join(b'\x00' + r for r in rows)

    def chunk(kind, body):
        return struct.pack('>I', len(body)) + kind + body + struct.pack('>I', zlib.crc32(kind + body) & 0xffffffff)
    with open(path, 'wb') as f:
        f.write(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0)) +
                chunk(b'IDAT', zlib.compress(raw, 6)) + chunk(b'IEND', b''))


def side_by_side(paths, out, gap=8):
    images = [read_png(p) for p in paths]
    height = max(h for _, h, _ in images)
    width = sum(w for w, _, _ in images) + gap * (len(images) - 1)
    spacer = b'\x20\x20\x20' * gap
    rows = []
    for y in range(height):
        parts = [rows_[y] if y < h else b'\x20\x20\x20' * w for w, h, rows_ in images]
        rows.append(spacer.join(parts))
    write_png(out, width, height, rows)


# ---- scenarios (test/coop/*.py: each defines run(coop), which raises to fail) ----

def run_tests(names):
    folder = os.path.join(REPO, 'test', 'coop')
    files = sorted(f for f in os.listdir(folder) if f.endswith('.py') and not f.startswith('_'))
    chosen = [f for f in files if not names or f[:-3] in names]
    if names and len(chosen) != len(names):
        raise CoopError('no scenario named %s (there are: %s)' % (
            ', '.join(n for n in names if n + '.py' not in files), ', '.join(f[:-3] for f in files)))
    failed = []
    for f in chosen:
        spec = importlib.util.spec_from_file_location('coop_' + f[:-3], os.path.join(folder, f))
        module = importlib.util.module_from_spec(spec)
        start = time.time()
        try:
            spec.loader.exec_module(module)
            module.run(sys.modules[__name__])
            print('PASS %s (%.1f s)' % (f[:-3], time.time() - start), flush=True)
        except (AssertionError, CoopError) as e:
            failed.append(f[:-3])
            print('FAIL %s: %s' % (f[:-3], e), flush=True)
        finally:
            Session.stop_named(f[:-3])
    print('%d of %d scenarios passed%s' % (len(chosen) - len(failed), len(chosen),
                                          ('; failed: ' + ', '.join(failed)) if failed else ''))
    return 1 if failed else 0


# ---- the command line ----

def _pairs(items):
    out = {}
    for item in items or []:
        key, _, value = item.partition('=')
        out[key] = value
    return out


def _print(name, reply):
    print('%s %s' % (name, json.dumps(reply, separators=(',', ':'))), flush=True)


def main(argv):
    if not argv or argv[0] in ('help', '-h', '--help'):
        print(__doc__)
        return 0
    name = 'default'
    if '--name' in argv:
        i = argv.index('--name')
        name = argv[i + 1]
        argv = argv[:i] + argv[i + 2:]
    verb = argv[0]
    try:
        if verb == 'launch':
            a = argparse.ArgumentParser(prog='coop.py launch')
            a.add_argument('--scene', default='track')
            a.add_argument('--players', type=int, default=2)
            a.add_argument('--delay', type=int, default=4)
            a.add_argument('--size', default='960x540')
            a.add_argument('--camera', action='append', help='pK=x,y,z,yawDeg,pitchDeg')
            a.add_argument('--script', action='append', help='pK=file: that player\'s commands (scenes/script.h)')
            a.add_argument('--bombard', type=int, default=0)
            a.add_argument('--running', action='store_true', help='the clock runs at 60 Hz instead of held')
            a.add_argument('--allow-input', action='store_true', help='real keys and mouse count too')
            o = a.parse_args(argv[1:])
            w, _, h = o.size.partition('x')
            s = Session.launch(name, o.scene, o.players, o.delay, (int(w), int(h)), _pairs(o.camera), _pairs(o.script),
                               o.bombard, o.running, o.allow_input)
            for p, st in zip(s.players, s.states()):
                _print(p.name, {'ok': True, 'port': p.port, 'pid': p.pid, 'tick': st['tick'], 'role': st['role'],
                                'session': st.get('session', {}).get('state', 'solo'), 'log': p.log})
            return 0
        if verb == 'test':
            return run_tests(argv[1:])
        s = Session.load(name)
        if verb == 'stop':
            s.stop()
            print('stopped %s' % name)
            return 0
        if verb == 'step':
            _print('p0', s.step(int(argv[1])))
            return 0
        if verb == 'turn':
            peer = s.player(argv[1]).peer
            reply = s.turn(peer, int(argv[2]))
            _print(argv[1], reply)
            _print(argv[1], s.player(argv[1]).state())
            return 0
        if verb == 'shot':
            which = next((x for x in argv[1:] if not x.startswith('--')), 'all')
            for path in s.shot(which, '--ui' in argv):
                print(path)
            return 0
        if verb == 'sync':
            states = s.states()
            for p, st in zip(s.players, states):
                _print(p.name, {'tick': st['tick'], 'hash': st['hash'], 'session': st.get('session', {}).get('state')})
            same = len({(st['tick'], st['hash']) for st in states}) == 1
            print('in sync' if same else 'NOT in sync')
            return 0 if same else 1
        if verb == 'log':
            print(s.player(argv[1]).tail(int(argv[2]) if len(argv) > 2 else 20), end='')
            return 0
        if verb == 'net':
            _print(argv[1], s.player(argv[1]).ask(' '.join(['net'] + argv[2:]), ok=False))
            return 0
        # A request to a player, or to all of them
        targets = s.players if verb == 'all' else [s.player(verb)]
        line = ' '.join(argv[1:])
        if not line:
            raise CoopError('no request given (coop.py help)')
        code = 0
        for p in targets:
            timeout = 600.0 if line.startswith(('wait', 'step', 'turn')) else 30.0
            reply = p.ask(line, timeout=timeout, ok=False)
            _print(p.name, reply)
            code = code if reply.get('ok') else 1
        return code
    except CoopError as e:
        print('error: %s' % e, file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
