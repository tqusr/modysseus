#!/usr/bin/env python3
"""Start the Saga AI mock server and play the organizer, route after route.

    source install/setup.bash
    python3 scripts/saga_mission_loop.py [--truck 1] [--dwell 10]

The robot never reports to Saga: routes advance only when an organizer
presses the admin buttons. This script presses them for you:

  1. starts the Saga server (generating credentials.json on first run),
     with uv if installed, else the server's own .venv,
  2. starts the event, which hands the truck its first route. The server
     saves the event to state.json and restores it on restart: delete that
     file for a fresh event,
  3. watches /dock_robot/_action/status. When a DockRobot goal succeeds it
     posts `arrived_at_<leg>`, and after `--dwell` seconds docked it posts
     `loading_complete` (source) or `unloading_complete` (destination).
     mission_node then undocks and drives on by itself. unloading_complete
     assigns the next route at once, so this repeats until Ctrl-C.

Start the robot stack separately, with the truck's token (printed below).
"""

import argparse
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path

import requests
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from rclpy.qos import qos_profile_action_status_default

ACTIVE = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
# The server unlocks the "complete" action ~3 s after the arrival action
# (`lock_seconds` in its config.toml).
MIN_DWELL_SEC = 4.0
# Route status -> the action that moves it on. ARRIVE is posted when the robot
# docks, COMPLETE after it has been docked for --dwell seconds.
ARRIVE = {'loading_pending': 'arrived_at_source',
          'loading_complete': 'arrived_at_destination'}
COMPLETE = {'loading_in_progress': 'loading_complete',
            'unloading_in_progress': 'unloading_complete'}


def log(message):
    print(f'[{time.strftime("%H:%M:%S")}] {message}', flush=True)


def server_python(server_dir):
    """uv if installed, else the server's own .venv, else this interpreter."""
    if shutil.which('uv'):
        return ['uv', 'run', 'python']
    venv_python = server_dir / '.venv' / 'bin' / 'python'
    return [str(venv_python)] if venv_python.exists() else [sys.executable]


def start_server(server_dir):
    python = server_python(server_dir)
    if not (server_dir / 'credentials.json').exists():
        log('No credentials.json, generating one')
        subprocess.run([*python, '-m', 'app.credentials', 'generate'],
                       cwd=server_dir, check=True)
    log(f'Starting Saga server in {server_dir}')
    return subprocess.Popen([*python, '-m', 'app'], cwd=server_dir)


def wait_for_server(url, server, timeout_sec=30.0):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if server is not None and server.poll() is not None:
            sys.exit(f'Saga server exited with code {server.returncode}')
        try:
            requests.get(f'{url}/healthz', timeout=1.0).raise_for_status()
            return
        except requests.RequestException:
            time.sleep(0.5)
    sys.exit(f'Saga server did not answer on {url} within {timeout_sec:.0f} s')


class Organizer:
    """Presses the admin page's buttons over HTTP and reads the truck's status."""

    def __init__(self, url, admin, truck_token, truck):
        self.url = url
        self.auth = (admin['username'], admin['password'])
        self.truck_headers = {'Authorization': f'Bearer {truck_token}'}
        self.truck = truck

    def post(self, path, data=None, retries=5):
        """Returns True once the server applied it.

        The admin endpoints answer 200 even when they refuse (wrong status, event
        not running); the refusal is in the HX-Trigger header as `saga-error`.
        A refusal is final and not retried; network errors and non-2xx answers are."""
        for attempt in range(1, retries + 1):
            try:
                response = requests.post(f'{self.url}{path}', data=data,
                                         auth=self.auth, timeout=2.0)
                if response.ok:
                    error = json.loads(response.headers.get('HX-Trigger') or '{}').get('saga-error')
                    if error:
                        log(f'Saga refused {path}: {error}')
                        return False
                    return True
                log(f'POST {path} -> {response.status_code} {response.text.strip()[:200]}')
            except (requests.RequestException, ValueError) as error:
                log(f'POST {path} failed: {error}')
            if attempt < retries:
                time.sleep(1.0)
        return False

    def action(self, name, expected_status):
        """`expected_status` makes the server reject it if the truck has
        already moved on, so a repeat can never be applied twice."""
        log(f'Truck {self.truck}: {name}')
        return self.post(f'/admin/trucks/{self.truck}/actions/{name}',
                         data={'expected_status': expected_status})

    def _get(self, path):
        try:
            response = requests.get(f'{self.url}{path}', headers=self.truck_headers, timeout=2.0)
            response.raise_for_status()
            return response.json()
        except (requests.RequestException, ValueError) as error:
            log(f'GET {path} failed: {error}')
            return None

    def status(self):
        """{'event_state', 'route_id', 'status', 'next_dock', 'completed_routes', ...}"""
        return self._get('/api/v1/status')

    def route(self):
        reply = self._get('/api/v1/route')
        return reply and reply.get('route')


def describe(route):
    if not route:
        return 'no route yet'
    return (f'route {route.get("route_id", "?")}: '
            f'{route["source"]["letter"]} -> {route["destination"]["letter"]}')


class DockWatcher:
    """Reports each DockRobot goal that goes from active to SUCCEEDED.

    Goals already finished at subscription (the status topic is latched) are
    ignored; only a goal seen active first counts as a fresh dock.
    """

    def __init__(self, node):
        self.active = set()
        self.docked = []
        node.create_subscription(GoalStatusArray, '/dock_robot/_action/status',
                                 self._on_status, qos_profile_action_status_default)

    def _on_status(self, message):
        for status in message.status_list:
            goal_id = bytes(status.goal_info.goal_id.uuid)
            if status.status in ACTIVE:
                self.active.add(goal_id)
            elif goal_id in self.active:
                self.active.discard(goal_id)
                if status.status == GoalStatus.STATUS_SUCCEEDED:
                    self.docked.append(time.monotonic())
                else:
                    log(f'DockRobot goal ended with status {status.status}, '
                        'mission_node will retry or safe-stop')


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--server-dir', type=Path,
                        default=Path.home() / 'ai-powered-autonomous-challenge-server')
    parser.add_argument('--url', default='http://localhost:8000')
    parser.add_argument('--no-server', action='store_true',
                        help='use a Saga server that is already running at --url')
    parser.add_argument('--credentials', type=Path,
                        help="the server's credentials.json "
                             '(default: <server-dir>/credentials.json)')
    parser.add_argument('--truck', default='1', help='truck number, 1-4')
    parser.add_argument('--dwell', type=float, default=10.0,
                        help='seconds docked before the load/unload is completed')
    args = parser.parse_args()
    dwell = max(args.dwell, MIN_DWELL_SEC)
    url = args.url.rstrip('/')
    credentials_path = args.credentials or args.server_dir / 'credentials.json'

    server = None
    if args.no_server and not credentials_path.is_file():
        sys.exit(f'{credentials_path} not found (pass --credentials)')
    if not args.no_server:
        if not args.server_dir.is_dir():
            sys.exit(f'Saga server not found at {args.server_dir} (pass --server-dir)')
        server = start_server(args.server_dir)
    try:
        wait_for_server(url, server)
        if not credentials_path.is_file():
            sys.exit(f'{credentials_path} not found (pass --credentials)')
        credentials = json.loads(credentials_path.read_text())
        organizer = Organizer(url, credentials['admin'], credentials['trucks'][args.truck],
                              args.truck)
        log('Saga server is up. Launch the robot (hardware.launch.py in another terminal) with:')
        log('  ros2 launch mission mission.launch.py saga_url:=<this host> '
            f"saga_token:=$(jq -r '.trucks[\"{args.truck}\"]' "
            f'{credentials_path})')

        rclpy.init()
        node = rclpy.create_node('saga_mission_loop')
        watcher = DockWatcher(node)

        status = organizer.status() or {}
        if status.get('event_state') != 'running':
            # Starts a new event, or resumes one restored from state.json.
            organizer.post('/admin/event/start')
            status = organizer.status() or {}
        log(f"Event {status.get('event_state')}, {describe(organizer.route())}, "
            f"status {status.get('status')}, {status.get('completed_routes')} routes done")

        # The server's route status decides every step, so a restart of this script
        # or the server (which restores state.json) resumes where the route is.
        docked_at = None
        if status.get('status') in COMPLETE:
            log(f"Already {status['status']}: completing in {dwell:.0f} s")
            docked_at = time.monotonic()
        while True:
            rclpy.spin_once(node, timeout_sec=0.2)
            if docked_at is None and watcher.docked:
                watcher.docked.clear()
                status = organizer.status() or {}
                arrival = ARRIVE.get(status.get('status'))
                if arrival is None:
                    log(f"Docked, but route status is {status.get('status')}: nothing to post")
                elif organizer.action(arrival, status['status']):
                    docked_at = time.monotonic()
                    log(f'Docked, completing in {dwell:.0f} s')
            if docked_at is not None and time.monotonic() - docked_at >= dwell:
                docked_at = None
                watcher.docked.clear()  # a dock reported mid-dwell is the same stop
                status = organizer.status() or {}
                completion = COMPLETE.get(status.get('status'))
                if completion is None:
                    log(f"Route status is {status.get('status')}, nothing to complete")
                elif organizer.action(completion, status['status']):
                    if completion == 'loading_complete':
                        log('Loaded, robot heads for the destination')
                    else:
                        done = (organizer.status() or {}).get('completed_routes')
                        log(f'Route done ({done} so far), next {describe(organizer.route())}')
    except KeyboardInterrupt:
        log('Stopping')
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        if server is not None:
            server.terminate()
            try:
                server.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                server.kill()


if __name__ == '__main__':
    main()
