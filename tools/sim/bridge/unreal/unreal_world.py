import ctypes
import functools
import multiprocessing
import numpy as np
import time

from multiprocessing import Pipe, Array
from openpilot.tools.sim.bridge.unreal.unreal_process import unreal_process, unreal_state
from openpilot.tools.sim.lib.common import SimulatorState, World
from openpilot.tools.sim.lib.camerad import W, H
from openpilot.tools.sim.lib.common import vec3

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

    self.unreal_process = multiprocessing.Process(name="unreal process",
          target=functools.partial(unreal_process, 
                self.camera_array, 
                self.wide_camera_array, 
                self.image_lock, 
                self.controls_recv, 
                self.state_send, 
                self.exit_event))

    self.unreal_process.start()

    # wait for a state message to ensure sim is launched
    #log.info('Waiting for the state message to ensure sim is launched')
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
    
    # TODO state receiving.
    # while self.state_recv.poll(0):
      
      # log.info('Reading the sensors ...')
      
      # Received state looks like this:
      # metadrive_state(velocity=vec3(x=0.007272727321833372, y=0.0, z=0), position=(5.0, 3.5), bearing=0.0, steering_angle=0)

      # TODO: Replace values in Unreal later
      # for now its hardcoded.

      # NO MD_STATE!!!
      # md_state = self.state_recv.recv()
      state.velocity = vec3(x=0.007272727321833372, y=0.0, z=0)
      state.bearing = 0.0
      state.steering_angle = 0
      state.gps.from_xy((5.0, 3.5))
      state.valid = True

  def read_cameras(self):
    pass

    
  def tick(self):
    pass

  def reset(self):
    self.should_reset = True

  def close(self):
    self.exit_event.set()
    self.unreal_process.join()
