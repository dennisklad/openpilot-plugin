import math
import numpy as np
from mss import mss
from PIL import Image
import time
import zmq
import io
import cv2

from collections import namedtuple
from panda3d.core import Vec3
from multiprocessing.connection import Connection

from openpilot.common.realtime import Ratekeeper

from openpilot.tools.sim.lib.common import vec3
from openpilot.tools.sim.lib.camerad import W, H

from threading import Thread

C3_POSITION = Vec3(0.0, 0, 1.22)
C3_HPR = Vec3(0, 0,0)

unity_state = namedtuple("unity_state", ["velocity", "position", "bearing", "steering_angle"])

def unity_process(camera_array, wide_camera_array, image_lock, controls_recv: Connection, state_send: Connection, exit_event):

  road_image = np.frombuffer(camera_array.get_obj(), dtype=np.uint8).reshape((H, W, 3))
  MAX_STEERING = 0
  print_rcv = ""

  def get_image():
    """Pulls the dashcam image from the socket. 
    The image is converted from bytes into an array with OpenCV.

    Returns:
        image
    """
    image_bytes = screen_socket.recv()
    nparr = np.frombuffer(image_bytes, np.uint8)
    return cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    
  
  def step(vc):
    """Executes a step by pushing the controls to the controls_socket.
    This string is then processed by the unity PULL socket.

    Args:
        vc (tuple): Both steering and acceleration values as +/-
    """
    steer, gas = vc
    control_commands = f"accelerate={gas}, steer={steer}"
    # Send control commands to Unity
    # print(f"Sending the controls to Unity... accelerate={gas:<5.3f}, steer={steer:3f}")
    print(f"Ctl to Unity: acc={gas:<6.3f}, str={steer:<6.3f} | State to OP: {print_rcv}")
    
    controls_socket.send_string(control_commands)
  
  def format_rcv_string(rcv):
    """This helper function generated a new string to display the unity state that is received by OP.
    This string has 1 floating point and consistent spacing.
    
    Args:
      rcv (string): The decoded string received from Unity
      
    Returns:
      string: The formatted string to be displayed

    """  
    l = []
    for e in rcv.split('|'):
      if '(' in e: # If tuple
        numbers = e.strip('()').split(',')
        new_tuple = '(' + ', '.join(f'{float(num):>4.0f}' for num in numbers) + ')'
        l.append(new_tuple)
      else: # If number
        l.append(f'{float(e):>6.1f}')
    return ' '.join(l)
  
  def get_state():
    """Threaded task that pulls from the state socket (pushed from unity).
    Decodes the unity state and created a unity_state named tuple.
    This is then send to the state_send pipe to unity_world.
    """
    state_socket = zmq.Context().socket(zmq.PULL)
    state_socket.setsockopt(zmq.CONFLATE, 1)
    state_socket.bind("tcp://127.0.0.1:5557")
    
    nonlocal MAX_STEERING # This value is required in the calculation for steer_unity
    nonlocal print_rcv  # This string is used to print to console in step
    
    while not exit_event.is_set():
      rcv = state_socket.recv().decode("utf-8") # Cast Byte Object to String
      
      print_rcv = format_rcv_string(rcv)
      
      # print(f"Receiving state from Unity...    " + rcv)

      # Example: b'(0.00, 0.00, 0.00)|(181.935, -333.5345)|0.0001210415|0.1|11'
      # 'vec3-velocity | position | heading_theta or bearing | steer | max steer'   
      state = rcv.split('|')
      
      # Check this value again.
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
  
  # TODO: Is this necessary? Does it work better without?
  # controls_socket.setsockopt(zmq.CONFLATE, 1)
  controls_socket.bind("tcp://127.0.0.1:5556")
  
  screen_socket = zmq.Context().socket(zmq.PULL)
  screen_socket.setsockopt(zmq.CONFLATE, 1)
  screen_socket.bind("tcp://127.0.0.1:5558")
  
  state_recv_thread = Thread(target=get_state)
  state_recv_thread.start()
    
  steer_ratio = 8
  vc = [0,0]
  print("exit_event.is_set():", exit_event.is_set())

  
  while not exit_event.is_set():
    
    if controls_recv.poll(0):
      while controls_recv.poll(0):
        
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
