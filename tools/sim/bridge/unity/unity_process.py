import math
import numpy as np
from mss import mss
from PIL import Image
import time
import zmq

from collections import namedtuple
from panda3d.core import Vec3
from multiprocessing.connection import Connection

from openpilot.common.realtime import Ratekeeper

from openpilot.tools.sim.lib.common import vec3
from openpilot.tools.sim.lib.camerad import W, H

from threading import Thread
import logging
log = logging.getLogger('a')

C3_POSITION = Vec3(0.0, 0, 1.22)
C3_HPR = Vec3(0, 0,0)

unity_state = namedtuple("unity_state", ["velocity", "position", "bearing", "steering_angle"])

def unity_process(camera_array, wide_camera_array, image_lock, controls_recv: Connection, state_send: Connection, exit_event):

  road_image = np.frombuffer(camera_array.get_obj(), dtype=np.uint8).reshape((H, W, 3))
  MAX_STEERING = 0

  def get_image():
      with mss() as sct:
        # log.info('Reading the cameras ...')
        # Get raw pixels from the screen
        # Main monitor is the second one detected. TODO: Add this as argument?
        main_monitor = sct.monitors[1]
        monitor = {"top": main_monitor['top']+100, "left": main_monitor['left']+100, "width": W, "height": H}
        sct_img = sct.grab(monitor)
        # TODO: Maybe better as Numpy array instead of PIL
        img = Image.frombytes("RGB", sct_img.size, sct_img.bgra, "raw", "BGRX")
        return np.array(img).reshape((H, W, 3))

  def step(vc):
    """Executes a step by pushing the controls to the controls_socket.
    This string is then processed by the unity PULL socket.

    Args:
        vc (tuple): Both steering and acceleration values as +/-
    """
    steer, gas = vc
    control_commands = f"accelerate={gas}, steer={steer}"
    # Send control commands to Unity
    log.info("Step: ZeroMQ: Sending the controls to Unity..." + control_commands)
    controls_socket.send_string(control_commands)
  
  
  def get_state():
    """Threaded task that pulls from the state socket (pushed from unity).
    Decodes the unity state and created a unity_state named tuple.
    This is then send to the state_send pipe to unity_world.
    """
    state_socket = zmq.Context().socket(zmq.PULL)
    state_socket.bind("tcp://127.0.0.1:5557")
    
    nonlocal MAX_STEERING # This value is required in the calculation for steer_unity
    
    while not exit_event.is_set():
      rcv = state_socket.recv().decode("utf-8") # Cast Byte Object to String
      
      log.info("State: ZeroMQ: Receiving state from Unity..." + rcv)

      # Example: b'(0.00, 0.00, 0.00)|(181.935, -333.5345)|0.0001210415|0.1|11'
      # 'vec3-velocity | position | heading_theta or bearing | steer | max steer'   
      state = rcv.split("|")
      
      # TODO: What is the bearing? Does it need math.degrees?
      # TODO: Steering values is not right.
      MAX_STEERING = int(state[-1])
      
      ustate = unity_state(
        velocity = vec3(x=eval(state[0])[0], y=eval(state[0])[1], z=eval(state[0])[2]),
        position = eval(state[1]),
        bearing  = float(state[2]),
        steering_angle = float(state[3]) * MAX_STEERING
      )
      
      state_send.send(ustate)

  #########################
  #       MAIN CODE       #
  #########################
  
  rk = Ratekeeper(100, None)

  # ZeroMQ Server Definition
  controls_socket = zmq.Context().socket(zmq.PUSH)
  controls_socket.bind("tcp://127.0.0.1:5556")
  
  state_recv_thread = Thread(target=get_state)
  state_recv_thread.start()
    
  steer_ratio = 8
  vc = [0,0]
  print("exit_event.is_set():", exit_event.is_set())

  
  while not exit_event.is_set():
    
    if controls_recv.poll(0):
      while controls_recv.poll(0):
        
          # TODO: Values are always 1 and -1...
          # print("Receiving controls...", controls_recv.recv())
          steer_angle, gas, should_reset = controls_recv.recv()

      steer_unity = steer_angle * 1 / (MAX_STEERING * steer_ratio)
      steer_unity = np.clip(steer_unity, -1, 1)

      vc = [steer_unity, gas]

    if rk.frame % 5 == 0:
      step(vc)
      road_image[...] = get_image()
      image_lock.release()

    rk.keep_time()
