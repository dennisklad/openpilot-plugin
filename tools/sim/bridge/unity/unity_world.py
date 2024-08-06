import ctypes
import functools
import multiprocessing
import numpy as np
import time

from openpilot.tools.sim.bridge.common import QueueMessage, QueueMessageType

from multiprocessing import Pipe, Array
from openpilot.tools.sim.bridge.unity.unity_process import (unity_process, unity_simulation_state, unity_vehicle_state)
from openpilot.tools.sim.lib.common import SimulatorState, World
from openpilot.tools.sim.lib.camerad import W, H


import logging
log = logging.getLogger('a')

class UnityWorld(World):
  def __init__(self, status_q, dual_camera=False):
    log.debug('Starting init of UnityWorld')
    super().__init__(dual_camera)

    self.status_q = status_q

    self.camera_array = Array(ctypes.c_uint8, W*H*3)
    self.road_image = np.frombuffer(self.camera_array.get_obj(), dtype=np.uint8).reshape((H, W, 3))

    self.wide_camera_array = None
    if dual_camera:
      self.wide_camera_array = Array(ctypes.c_uint8, W*H*3)
      self.wide_road_image = np.frombuffer(self.wide_camera_array.get_obj(), dtype=np.uint8).reshape((H, W, 3))

    self.controls_send, self.controls_recv = Pipe()
    self.simulation_state_send, self.simulation_state_recv = Pipe()
    self.vehicle_state_send, self.vehicle_state_recv = Pipe()


    self.exit_event = multiprocessing.Event()

    self.unity_process = multiprocessing.Process(name="unity process",
          target=functools.partial(unity_process,
                dual_camera,
                self.camera_array,
                self.wide_camera_array,
                self.image_lock,
                self.controls_recv,
                self.simulation_state_send,
                self.vehicle_state_send,
                self.exit_event))

    self.unity_process.start()
    self.status_q.put(QueueMessage(QueueMessageType.START_STATUS, "starting"))

    # wait for a state message to ensure sim is launched
    log.info('Waiting for the state message to ensure sim is launched')
    self.vehicle_state_recv.recv()

    self.status_q.put(QueueMessage(QueueMessageType.START_STATUS, "started"))

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

    while self.vehicle_state_recv.poll(0):
      # log.info('Reading the state of the sensors ...' + str(self.state_recv.recv()))
      temp_state: unity_vehicle_state = self.vehicle_state_recv.recv()

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

  def close(self, reason):
    self.status_q.put(QueueMessage(QueueMessageType.CLOSE_STATUS, reason))
    self.exit_event.set()

    self.unity_process.join()
