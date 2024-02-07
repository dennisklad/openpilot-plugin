import math
import numpy as np
from mss import mss
from PIL import Image

from collections import namedtuple
from panda3d.core import Vec3
from multiprocessing.connection import Connection

from openpilot.common.realtime import Ratekeeper

from openpilot.tools.sim.lib.common import vec3
from openpilot.tools.sim.lib.camerad import W, H

C3_POSITION = Vec3(0.0, 0, 1.22)
C3_HPR = Vec3(0, 0,0)

unreal_state = namedtuple("unreal_state", ["velocity", "position", "bearing", "steering_angle"])


def unreal_process(camera_array, wide_camera_array, image_lock, controls_recv: Connection, state_send: Connection, exit_event):

  road_image = np.frombuffer(camera_array.get_obj(), dtype=np.uint8).reshape((H, W, 3))

  def get_image():
      with mss() as sct:
        # log.info('Reading the cameras ...')
        # Get raw pixels from the screen
        monitor = {"top": 0, "left": 0, "width": W, "height": H}
        sct_img = sct.grab(monitor)
        # TODO: Maybe better as Numpy array instead of PIL
        img = Image.frombytes("RGB", sct_img.size, sct_img.bgra, "raw", "BGRX")
        return np.array(img).reshape((H, W, 3))

  rk = Ratekeeper(100, None)

  steer_ratio = 8
  vc = [0,0]
  print("FUUUUCK:", exit_event.is_set())
  while not exit_event.is_set():
    # state = unreal_state(
    #   velocity=vec3(x=float(env.vehicle.velocity[0]), y=float(env.vehicle.velocity[1]), z=0),
    #   position=env.vehicle.position,
    #   bearing=float(math.degrees(env.vehicle.heading_theta)),
    #   steering_angle=env.vehicle.steering * env.vehicle.MAX_STEERING
    # )

    # state_send.send(state)

    # if controls_recv.poll(0):
    #   while controls_recv.poll(0):
    #     steer_angle, gas, should_reset = controls_recv.recv()

    #   steer_unreal = steer_angle * 1 / (env.vehicle.MAX_STEERING * steer_ratio)
    #   steer_unreal = np.clip(steer_unreal, -1, 1)

    #   vc = [steer_unreal, gas]

    if rk.frame % 5 == 0:
      road_image[...] = get_image()
      image_lock.release()

    rk.keep_time()
