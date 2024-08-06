#!/usr/bin/env python
import argparse

from typing import Any
from multiprocessing import Queue

from openpilot.tools.sim.bridge.unity.unity_bridge import UnityBridge
from openpilot.tools.sim.bridge.metadrive.metadrive_bridge import MetaDriveBridge

import logging
log = logging.getLogger('a')
log.setLevel(logging.DEBUG)

def create_bridge(dual_camera, high_quality):
  queue: Any = Queue()

  if args.sim == 'metadrive':
    simulator_bridge = MetaDriveBridge(dual_camera, high_quality)
    log.debug('`MetaDriveBridge.run(q)` called. Init Bridge.')

  else:
    simulator_bridge = UnityBridge(dual_camera, high_quality, queue)
    log.debug('`UnityBridge.run(q)` called. Init Bridge.')

  simulator_process, driving_prms_process = simulator_bridge.run(queue)

  return queue, simulator_process, driving_prms_process, simulator_bridge

def main():
  _, simulator_process, _ = create_bridge(True, False)
  simulator_process.join()

def parse_args(add_args=None):
  parser = argparse.ArgumentParser(description='Bridge between the simulator and openpilot.')
  parser.add_argument('--joystick', action='store_true')
  parser.add_argument('--high_quality', action='store_true')
  parser.add_argument('--dual_camera', action='store_true')
  parser.add_argument('--sim')

  return parser.parse_args(add_args)

if __name__ == "__main__":

  args = parse_args()

  # <------ 1) Calls the run function of unityBridge
  queue, simulator_process, driving_prms_process, simulator_bridge = create_bridge(args.dual_camera, args.high_quality)

  if args.joystick:
    # start input poll for joystick
    from openpilot.tools.sim.lib.manual_ctrl import wheel_poll_thread

    wheel_poll_thread(queue)
  else:
    # start input poll for keyboard
    from openpilot.tools.sim.lib.keyboard_ctrl import keyboard_poll_thread

    keyboard_poll_thread(queue)

  simulator_bridge.shutdown()

  # Ensure all processes are joined
  driving_prms_process.join()
  simulator_process.join()