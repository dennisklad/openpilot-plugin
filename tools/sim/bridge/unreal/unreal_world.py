import ctypes
import functools
import multiprocessing
import numpy as np
import time

from multiprocessing import Pipe, Array
from openpilot.tools.sim.lib.common import SimulatorState, World
from openpilot.tools.sim.lib.camerad import W, H

from mss import mss
from PIL import Image

import logging
log = logging.getLogger('a')

class UnrealWorld(World):
  
  def __init__(self):
    
    log.debug('Starting init of UnrealWorld')

    super().__init__(dual_camera=False)

    self.camera_array = Array(ctypes.c_uint8, W*H*3)
  
    # TODO ADD SCREENSHOT HERE!
    self.road_image = np.frombuffer(self.camera_array.get_obj(), dtype=np.uint8).reshape((H, W, 3))

    self.wide_camera_array = None
    
    self.controls_send, self.controls_recv = Pipe()
    self.state_send, self.state_recv = Pipe()

    self.exit_event = multiprocessing.Event()

    # wait for a state message to ensure sim is launched
    log.info('Waiting for the state message to ensure sim is launched')
    #self.state_recv.recv() 

    self.steer_ratio = 15
    self.vc = [0.0,0.0]
    self.reset_time = 0
    self.should_reset = False

    log.debug('UnrealWorld initiated.')

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

    self.controls_send.send([*self.vc, self.should_reset])
    self.should_reset = False

  def read_sensors(self, state: SimulatorState):
    while self.state_recv.poll(0):
      md_state = self.state_recv.recv()
      state.velocity = md_state.velocity
      state.bearing = md_state.bearing
      state.steering_angle = md_state.steering_angle
      state.gps.from_xy(md_state.position)
      state.valid = True

  def read_cameras(self):
    with mss() as sct:
      # Get raw pixels from the screen
      monitor = {"top": 0, "left": 0, "width": W, "height": H}
      sct_img = sct.grab(monitor)

      # TODO: Maybe better as Numpy array instead of PIL
      img = Image.frombytes("RGB", sct_img.size, sct_img.bgra, "raw", "BGRX")

      return np.array(img).reshape((W,H,3))
    
  def tick(self):
    pass

  def reset(self):
    self.should_reset = True

  def close(self):
    self.exit_event.set()
