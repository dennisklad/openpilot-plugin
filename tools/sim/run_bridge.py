#!/usr/bin/env python
import argparse

from typing import Any
from multiprocessing import Queue

from openpilot.tools.sim.bridge.unity.unity_bridge import UnityBridge
from openpilot.tools.sim.bridge.metadrive.metadrive_bridge import MetaDriveBridge

import logging
log = logging.getLogger('a')
log.setLevel(logging.DEBUG)

def parse_args(add_args=None):
  parser = argparse.ArgumentParser(description='Bridge between the simulator and openpilot.')
  parser.add_argument('--joystick', action='store_true')
  parser.add_argument('--high_quality', action='store_true')
  parser.add_argument('--dual_camera', action='store_true')
  parser.add_argument('--sim')

  return parser.parse_args(add_args)

if __name__ == "__main__":

  q: Any = Queue()
  args = parse_args()

  if args.sim == 'metadrive':
    simulator_bridge = MetaDriveBridge(args)
    log.debug('`MetaDriveBridge.run(q)` called. Init Bridge.')

  else:
    simulator_bridge = UnityBridge(args)
    log.debug('`UnityBridge.run(q)` called. Init Bridge.')
    # <------ 1) Calls the run function of unityBridge

  p = simulator_bridge.run(q)                 

  if args.joystick:
    # start input poll for joystick
    from openpilot.tools.sim.lib.manual_ctrl import wheel_poll_thread

    wheel_poll_thread(q)
  else:
    # start input poll for keyboard
    from openpilot.tools.sim.lib.keyboard_ctrl import keyboard_poll_thread

    keyboard_poll_thread(q)

  simulator_bridge.shutdown()

  p.join()
