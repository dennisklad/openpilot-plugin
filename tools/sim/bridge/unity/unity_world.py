import ctypes
import functools
import multiprocessing
import numpy as np
import time

from multiprocessing import Pipe, Array
from openpilot.tools.sim.bridge.unity.unity_process import unity_process, unity_state
from openpilot.tools.sim.lib.common import SimulatorState, World
from openpilot.tools.sim.lib.camerad import W, H
from openpilot.tools.sim.lib.common import vec3

import logging
log = logging.getLogger('a')

class UnityWorld(World):

  def __init__(self, queue):

    log.debug('Starting init of UnityWorld')

    super().__init__(dual_camera=False)

    self.camera_array = Array(ctypes.c_uint8, W*H*3)

    self.road_image = np.frombuffer(self.camera_array.get_obj(), dtype=np.uint8).reshape((H, W, 3))

    self.controls_send, self.controls_recv = Pipe()
    self.state_send, self.state_recv = Pipe()

    self.exit_event = multiprocessing.Event()

    self.unity_process = multiprocessing.Process(name="unity process",
          target=functools.partial(unity_process,
                self.camera_array,
                self.image_lock,
                self.controls_recv,
                self.state_send,
                self.exit_event))

    self.unity_process.start()

    # wait for a state message to ensure sim is launched
    log.info('Waiting for the state message to ensure sim is launched')
    self.state_recv.recv()

    self.steer_ratio = 15
    self.vc = [0.0,0.0]
    self.reset_time = 0
    self.should_reset = False

    log.debug('UnityWorld initiated.')

  def apply_controls(self, steer_angle, throttle_out, brake_out):
    if (time.monotonic() - self.reset_time) > 2:
      self.vc[0] = steer_angle

      if throttle_out:
        self.vc[1] = throttle_out
      else:
        self.vc[1] = -brake_out
    else:
      self.vc[0] = 0
      self.vc[1] = 0

    # Send the controls to the pipe
    self.controls_send.send([*self.vc, self.should_reset])
    self.should_reset = False

  def read_state(self):
    pass

  def read_sensors(self, state: SimulatorState):

    while self.state_recv.poll(0):
      # log.info('Reading the state of the sensors ...' + str(self.state_recv.recv()))
      temp_state: unity_state = self.state_recv.recv()
      state.velocity = temp_state.velocity
      state.bearing = temp_state.bearing
      state.steering_angle = temp_state.steering_angle
      state.gps.from_xy(temp_state.position)
      state.valid = True

  def read_cameras(self):
    pass


  def tick(self):
    pass

  def reset(self):
    self.should_reset = True

  def close(self):
    self.exit_event.set()

    # TODO: Close correctly
    self.unity_process.join()
