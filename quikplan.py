#!/usr/bin/env python3

import importlib
import sys
import argparse

from robot import Robot

parser = argparse.ArgumentParser()

parser.add_argument("--init", action="store_true", help="Launch the initialization GUI")
parser.add_argument("--slalom", action="store_true", help="Run the slalom optimizer")
parser.add_argument(
    "--barrel_racing", action="store_true", help="Run the barrel racing optimizer"
)

args = parser.parse_args()

if args.init:
    init = importlib.import_module("quikgui")
    init.run()
elif args.slalom:
    slalom = importlib.import_module("quikplan_slalom")
    slalom.plan(plot=True)
elif args.barrel_racing:
    barrel_racing = importlib.import_module("quikplan_barrel_racing")
    barrel_racing.plan(plot=True)
