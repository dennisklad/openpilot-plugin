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

import logging
log = logging.getLogger('a')

C3_POSITION = Vec3(0.0, 0, 1.22)
C3_HPR = Vec3(0, 0,0)

unity_state = namedtuple("unity_state", ["velocity", "position", "bearing", "steering_angle"])

def unity_process(camera_array, wide_camera_array, image_lock, controls_recv: Connection, state_send: Connection, exit_event):

  road_image = np.frombuffer(camera_array.get_obj(), dtype=np.uint8).reshape((H, W, 3))

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
    steer, gas = vc
    # Simulate control commands (for demonstration)
    control_commands = f"accelerate={gas}, steer={steer}"
    # Send control commands to Unity
    # log.info("ZeroMQ: Sending the controls to Unity...")
    # socket.send_string(control_commands)
    # print("ZeroMQ: Response from Unity about the controls:", socket.recv_string())
  
  
  def get_state():
    # TODO: Current state of the car in Unity!!!
    # TODO: Make it non-blocking (?)
    # state = unity_state(
    #   # Get values from the bridge
    #   velocity = vec3(x=float(vehicle.velocity[0]), y=float(vehicle.velocity[1]), z=0),
    #   position = vehicle.position,
    #   bearing  = float(math.degrees(vehicle.heading_theta)),
    #   steering_angle = vehicle.steering * vehicle.MAX_STEERING
    # )
    
    # log.info("ZeroMQ: Receiving state from Unity...")
    rcv = socket.recv()
    log.info(rcv)
    socket.send(b"Okay")
    # return rcv


  #########################
  #       MAIN CODE       #
  #########################
  
  rk = Ratekeeper(100, None)

  # ZeroMQ Server Definition
  server_address = "tcp://127.0.0.1:5555"
  socket = zmq.Context().socket(zmq.REP)
  socket.bind(server_address)
  log.debug(f"ZeroMQ Server startet at {server_address}...")
  
  steer_ratio = 8
  vc = [0,0]
  print("exit_event.is_set():", exit_event.is_set())

  while not exit_event.is_set():
    
    get_state()
    # state_send.send(get_state())
    
    if controls_recv.poll(0):
      while controls_recv.poll(0):
        
          # print("Receiving controls...")
          # print(controls_recv.recv())
          steer_angle, gas, should_reset = controls_recv.recv()

      # steer_unity = steer_angle * 1 / (MAX_STEERING * steer_ratio)
      # steer_unity = np.clip(steer_unity, -1, 1)

      # vc = [steer_unity, gas]

    if rk.frame % 5 == 0:
      step(vc)       # TODO: Call the function
      road_image[...] = get_image()
      image_lock.release()

    rk.keep_time()
