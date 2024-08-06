import numpy as np
import zmq
import cv2
import vgamepad as vg

from collections import namedtuple
from panda3d.core import Vec3
from multiprocessing.connection import Connection
from openpilot.common.realtime import Ratekeeper

from openpilot.tools.sim.lib.common import vec3
from openpilot.tools.sim.lib.camerad import W, H

from threading import Thread

import time

C3_POSITION = Vec3(0.0, 0, 1.22)
C3_HPR = Vec3(0, 0,0)

unity_simulation_state = namedtuple("unity_simulation_state", ["running", "done", "done_info"])
unity_vehicle_state = namedtuple("unity_vehicle_state", ["velocity", "position", "bearing", "steering_angle"])

def unity_process(dual_camera: bool, camera_array, wide_camera_array, image_lock,
                  controls_recv: Connection, simulation_state_send: Connection, vehicle_state_send: Connection, exit_event):

  road_image = np.frombuffer(camera_array.get_obj(), dtype=np.uint8).reshape((H, W, 3))
  if dual_camera:
    assert wide_camera_array is not None
    wide_road_image = np.frombuffer(wide_camera_array.get_obj(), dtype=np.uint8).reshape((H, W, 3))

  MAX_STEERING = 0
  is_engaged = False
  print_rcv = ""
  steer_angle = 0
  is_reverse = False
  is_handbrake = False

  def get_image(cam):
    """Pulls the dashcam image from the socket.

    The image is converted from bytes into an array with OpenCV.

    Returns:
        image
    """
    if cam == "rgb_road":
      image_bytes = screen_socket_road.recv()
      nparr = np.frombuffer(image_bytes, np.uint8)
      return cv2.imdecode(nparr, cv2.IMREAD_COLOR)

    elif cam == "rgb_wide":
      image_bytes = screen_socket_wide.recv()
      nparr = np.frombuffer(image_bytes, np.uint8)
      return cv2.imdecode(nparr, cv2.IMREAD_COLOR)

  def step(vc):
    """Executes a step by pushing the controls to the controls_socket.
    This string is then processed by the unity PULL socket.

    Args:
        vc (tuple): Both steering and acceleration values as [-1; 1]
    """
    steer, gas = vc

    print(f"Ctl to Unity: acc={gas:>6.3f}, str={steer:>6.3f} | State to OP: {print_rcv}")

    if is_engaged:
      control_gamepad(gas, steer)

    else:
      control_gamepad(0,0)

  def control_gamepad(gas:float, steer:float):
    """This function changes the left-stick and presses buttons on the virtual gamepad.

    Args:
        gas   (float): Value to accelerate or brake [-1, 1]
        steer (float): Value to steer left or right [-1, 1]
    """
    nonlocal is_handbrake

    # The small steering values are not doing anything in Unity
    # Boost small adjustments to move the car sligthly
    if abs(steer) < 0.1:
      steer *= 4

    # Release the handbreak if it's a clear drive indication
    if is_handbrake and gas > 0.5:
      is_handbrake = False
      send_driving_instr_socket.send_string('release_handbrake')
      print("releasing the handbrake...")

    # Ignore inputs will in parking mode
    elif is_handbrake and gas <= 0.5:
      print("in parking...")
      # Reset the joystick
      gamepad.left_joystick_float(0, 0)
      gamepad.update()

    # Block OP from going in reverse
    elif gas < 0 and is_reverse:
      print("setting the handbrake...")

      # Reset the joystick
      gamepad.left_joystick_float(0, 0)
      gamepad.update()

      # Set the handbrake
      send_driving_instr_socket.send_string('set_handbrake')
      is_handbrake = True

    # If not reversing or parking apply the joystick controls
    else:
      gamepad.left_joystick_float(-steer, -gas)
      gamepad.update()


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
        l.append(f'{int(float(e)):>3}')
    return ' '.join(l)


  def get_state():
    """Threaded task that pulls from the state socket (pushed from unity).
    Decodes the unity state and created a unity_vehicle_state named tuple.
    This is then send to the state_send pipe to unity_world.
    """
    rcv_state_socket = zmq.Context().socket(zmq.PULL)
    rcv_state_socket.setsockopt(zmq.CONFLATE, 1)
    rcv_state_socket.bind("tcp://127.0.0.1:5556")

    nonlocal MAX_STEERING # This value is required in the calculation for steer_unity
    nonlocal print_rcv    # This string is used to print to console in step
    nonlocal steer_angle  # This value is received from the internal controls pipe
    nonlocal is_engaged   # This bool is used to determine when the virtual controller should be active
    nonlocal is_reverse   # This bool is used to determine if the car is driving in reverse

    while not exit_event.is_set():
      rcv = rcv_state_socket.recv().decode("utf-8") # Cast Byte Object to String
      print_rcv = format_rcv_string(rcv)

      # Example: b'(0.00, 0.00, 0.00)|(181.935, -333.5345)|0.1|11|0|0'
      # 'vec3-velocity | position | heading_theta or bearing | max steer | is_engaged | is_reversing'
      state = rcv.split('|')

      MAX_STEERING = int(state[3])
      is_engaged = bool(state[4] == '1')
      is_reverse = bool(state[5] == '1')
      vehicle_state = unity_vehicle_state(
        velocity = vec3(x=eval(state[0])[0], y=eval(state[0])[1], z=eval(state[0])[2]),
        position = eval(state[1]),
        bearing  = float(state[2]),
        steering_angle = steer_angle * MAX_STEERING,
      )

      vehicle_state_send.send(vehicle_state)


  ############################
  # ZeroMQ Server Definition #
  ############################

  # Socket to receive the road camera frame from Unity
  screen_socket_road = zmq.Context().socket(zmq.PULL)
  screen_socket_road.setsockopt(zmq.CONFLATE, 1)
  screen_socket_road.bind("tcp://127.0.0.1:5557")

  # Socket to receive the wide camera frame from Unity
  if dual_camera:
    screen_socket_wide = zmq.Context().socket(zmq.PULL)
    screen_socket_wide.setsockopt(zmq.CONFLATE, 1)
    screen_socket_wide.bind("tcp://127.0.0.1:5558")

  # Socket to receive the vehicle state from Unity
  state_recv_thread = Thread(target=get_state)
  state_recv_thread.start()

  # Socket to send the potential driving instructions to Unity
  send_driving_instr_socket = zmq.Context().socket(zmq.PUSH)
  send_driving_instr_socket.connect("tcp://127.0.0.1:5560")


  #########################
  #       MAIN CODE       #
  #########################

  simulation_state = unity_simulation_state(
    running=True,
    done=False,
    done_info=None,
  )
  simulation_state_send.send(simulation_state)
        
  rk = Ratekeeper(100, None)

  # Define the virtual gamepad to control unity
  gamepad = vg.VX360Gamepad()

  steer_ratio = 8
  vc = [0,0]

  send_driving_instr_socket.send_string('connected')

  while not exit_event.is_set():

    if controls_recv.poll(0):
      while controls_recv.poll(0):
        steer_angle, gas, _ = controls_recv.recv()

      try:
        steer_unity = steer_angle * 1 / (MAX_STEERING * steer_ratio)
        steer_unity = np.clip(steer_unity, -1, 1)
      except Exception as e:
        print(e)
        steer_unity = 0

      vc = [steer_unity, gas]

    if rk.frame % 5 == 0:

      # Do the necessary steps for the accl and steer received from the OP model
      step(vc)

      road_image[...] = get_image("rgb_road")

      if dual_camera:
        wide_road_image[...] = get_image("rgb_wide")


      image_lock.release()

    rk.keep_time()
