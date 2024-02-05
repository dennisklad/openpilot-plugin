import signal
import threading
import functools

from multiprocessing import Process, Queue
from abc import ABC, abstractmethod
from typing import Optional

from openpilot.common.params import Params
from openpilot.common.numpy_fast import clip
from openpilot.common.realtime import Ratekeeper
from openpilot.selfdrive.test.helpers import set_params_enabled
from openpilot.selfdrive.car.honda.values import CruiseButtons
from openpilot.tools.sim.lib.common import SimulatorState, World
from openpilot.tools.sim.lib.simulated_car import SimulatedCar
from openpilot.tools.sim.lib.simulated_sensors import SimulatedSensors
from openpilot.tools.sim.lib.keyboard_ctrl import KEYBOARD_HELP

import logging
log = logging.getLogger('a')

def rk_loop(function, hz, exit_event: threading.Event):
  rk = Ratekeeper(hz, None)
  while not exit_event.is_set():
    function()
    rk.keep_time()

class SimulatorBridge(ABC):
  TICKS_PER_FRAME = 5

  def __init__(self, arguments):
    set_params_enabled()
    self.params = Params()

    self.rk = Ratekeeper(100, None)

    self.dual_camera = arguments.dual_camera
    self.high_quality = arguments.high_quality

    self._exit_event = threading.Event()
    self._threads = []
    self._keep_alive = True
    self.started = False
    signal.signal(signal.SIGTERM, self._on_shutdown)
    self._exit = threading.Event()
    self.simulator_state = SimulatorState()

    self.world: Optional[World] = None

    self.past_startup_engaged = False

  def _on_shutdown(self, signal, frame):
    self.shutdown()

  def shutdown(self):
    self._keep_alive = False

  def bridge_keep_alive(self, q: Queue, retries: int):              
    # <----- 4) Launch _run Function with Queue
    log.info('`bridge_keep_alive(q)` called. Starting `_run(q)`.')
    try:
      self._run(q)
    finally:
      self.close()

  def close(self):
    self.started = False
    self._exit_event.set()

    if self.world is not None:
      self.world.close()

  def run(self, queue, retries=-1):                                
    # <----- 3) Start bridge_keep_alive Process
    log.info('`run(q)` called. Starting the `bridge_keep_alive` process.')
    bridge_p = Process(name="bridge", target=self.bridge_keep_alive, args=(queue, retries))  
    bridge_p.start()
    return bridge_p

  def print_status(self):
    print(
    f"""
Keyboard Commands:
{KEYBOARD_HELP}

State:
Ignition: {self.simulator_state.ignition} Engaged: {self.simulator_state.is_engaged}
    """)

  @abstractmethod
  def spawn_world(self) -> World:                                 
    # Overwritten in metadrive_bridge.py
    pass

  def log_sm(self):
        line = '\n\n---------------------------------------------------------------------------------\n\n'
        print('Hello'+line)
        print('controlState:', self.simulated_car.sm['controlsState'] + line)
        print('carControl',  self.simulated_car.sm['carControl'] + line)
        print('carParams', self.simulated_car.sm['carParams'] + line)

  def _run(self, q: Queue):
    
    log.debug('`_run(q)` helper function called.')
    # <----- 5) Spawn World, Car, Sensors and run the simulation
    
    self.world = self.spawn_world()
    log.info('`spawn_world` completed.')

    self.simulated_car = SimulatedCar()
    log.info('Spawning the SimulatedCar.')

    self.simulated_sensors = SimulatedSensors(self.dual_camera)
    log.info('Spawning the SimulatedSensors.')

    self.simulated_car_thread = threading.Thread(
      target=rk_loop, args=(functools.partial(self.simulated_car.update, self.simulator_state), 100, self._exit_event))
    self.simulated_car_thread.start()

    self.simulated_camera_thread = threading.Thread(
      target=rk_loop, args=(functools.partial(self.simulated_sensors.send_camera_images, self.world), 20, self._exit_event))
    self.simulated_camera_thread.start()

    # Simulation tends to be slow in the initial steps. This prevents lagging later
    for _ in range(20):
      self.world.tick()

    print('Alive')
    while self._keep_alive:
      throttle_out = steer_out = brake_out = 0.0
      throttle_op = steer_op = brake_op = 0.0

      self.simulator_state.cruise_button = 0
      self.simulator_state.left_blinker = False
      self.simulator_state.right_blinker = False

      throttle_manual = steer_manual = brake_manual = 0.

      # Read manual controls
      if not q.empty():
        message = q.get()
        m = message.split('_')
        if m[0] == "steer":
          steer_manual = float(m[1])
        elif m[0] == "throttle":
          throttle_manual = float(m[1])
        elif m[0] == "brake":
          brake_manual = float(m[1])
        elif m[0] == "cruise":
          if m[1] == "down":
            self.simulator_state.cruise_button = CruiseButtons.DECEL_SET
          elif m[1] == "up":
            self.simulator_state.cruise_button = CruiseButtons.RES_ACCEL
          elif m[1] == "cancel":
            self.simulator_state.cruise_button = CruiseButtons.CANCEL
          elif m[1] == "main":
            self.simulator_state.cruise_button = CruiseButtons.MAIN
        elif m[0] == "blinker":
          if m[1] == "left":
            self.simulator_state.left_blinker = True
          elif m[1] == "right":
            self.simulator_state.right_blinker = True
        elif m[0] == "ignition":
          self.simulator_state.ignition = not self.simulator_state.ignition
        elif m[0] == "reset":
          self.world.reset()
        elif m[0] == "quit":
          break

      self.simulator_state.user_brake = brake_manual
      self.simulator_state.user_gas = throttle_manual
      self.simulator_state.user_torque = steer_manual * -10000

      steer_manual = steer_manual * -40

      # Update openpilot on current sensor state
      # log.debug('Updating the `simulated_sensor`')
      self.simulated_sensors.update(self.simulator_state, self.world)

      self.simulated_car.sm.update(0)     # <---- Update the simcar through the SubMaster
      controlsState = self.simulated_car.sm['controlsState']
      self.simulator_state.is_engaged = controlsState.active

      if self.simulator_state.is_engaged:
        throttle_op = clip(self.simulated_car.sm['carControl'].actuators.accel / 1.6, 0.0, 1.0)
        brake_op = clip(-self.simulated_car.sm['carControl'].actuators.accel / 4.0, 0.0, 1.0)
        steer_op = self.simulated_car.sm['carControl'].actuators.steeringAngleDeg

        self.past_startup_engaged = True
      elif not self.past_startup_engaged and controlsState.engageable:
        self.simulator_state.cruise_button = CruiseButtons.DECEL_SET # force engagement on startup

      throttle_out = throttle_op if self.simulator_state.is_engaged else throttle_manual
      brake_out = brake_op if self.simulator_state.is_engaged else brake_manual
      steer_out = steer_op if self.simulator_state.is_engaged else steer_manual

      self.world.apply_controls(steer_out, throttle_out, brake_out)
      self.world.read_sensors(self.simulator_state)

      if self.rk.frame % self.TICKS_PER_FRAME == 0:
        self.world.tick()
        self.world.read_cameras()

      if self.rk.frame % 25 == 0:
        self.print_status()

      self.started = True

      self.rk.keep_time()

      #self.log_sm()


