#!/usr/bin/env python3
"""Rewrite perception/config/docking.yaml's dock poses from mission/config/docks.yaml.

docks.yaml is the source of truth for where the docks are; docking.yaml carries
the same docks in opennav_docking's schema (robot nose-forward heading, moved
0.126 m out from the wall). `capture_dock_pose` writes both, but a dock captured
some other way (or a map change) leaves docking.yaml pointing at a different
arena, and tag_dock_server then falls back to those poses. Run this after any
change to docks.yaml:

    scripts/sync_docking_yaml.py [--check]

--check only reports (exit 1 when out of sync); nothing is written.
"""
import argparse
import os
import sys

import yaml

WS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(WS, 'src/perception'))
from perception.dock_capture import (DockCapture, Pose2D, docking_server_pose_for,  # noqa: E402
                                     format_docking_entry, replace_entry_line)

DOCKS = os.path.join(WS, 'src/mission/config/docks.yaml')
DOCKING = os.path.join(WS, 'src/perception/config/docking.yaml')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--check', action='store_true')
    args = ap.parse_args()
    docks = yaml.safe_load(open(DOCKS))['docks']
    text = open(DOCKING).read()
    new = text
    for letter, d in docks.items():
        dp, sp = d['dock_pose'], d['staging_pose']
        cap = DockCapture(letter, d['tag_id'], Pose2D(dp['x'], dp['y'], dp['theta']),
                          Pose2D(sp['x'], sp['y'], sp['theta']),
                          docking_server_pose_for(Pose2D(dp['x'], dp['y'], dp['theta']), 0.126))
        new = replace_entry_line(new, letter, format_docking_entry(cap))
    if new == text:
        print('docking.yaml already matches docks.yaml')
        return 0
    if args.check:
        print('docking.yaml is OUT OF SYNC with docks.yaml')
        return 1
    open(DOCKING, 'w').write(new)
    print('docking.yaml updated:')
    return 0


if __name__ == '__main__':
    sys.exit(main())
