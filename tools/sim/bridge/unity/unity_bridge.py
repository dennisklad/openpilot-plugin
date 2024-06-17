from openpilot.tools.sim.bridge.common import SimulatorBridge, control_cmd_gen
from openpilot.tools.sim.bridge.unity.unity_world import UnityWorld
from openpilot.tools.sim.lib.camerad import W, H
from openpilot.tools.sim.lib.common import World
from multiprocessing import Process, Queue

import zmq

import logging
log = logging.getLogger('a')

class UnityBridge(SimulatorBridge):
  TICKS_PER_FRAME = 5

  def __init__(self, dual_camera, high_quality, queue):

    self.queue = queue
    super(UnityBridge, self).__init__(dual_camera, high_quality)   # <-------  2) Initialise SimulatorBridge

  def spawn_world(self, queue: Queue) -> World:
    """This function is called once when initiating the bridge.
    The call is found in the _run function of /tools/sim/bridge/common.py

    Returns:
        World:
    """
    log.debug("`UnityBridge.spawn_world` called. Generating empty UnityWorld.")
    return UnityWorld(queue)

  def get_driving_prms(self, queue: Queue):
    """This function requires access to the queue in order to add the parsed driving instructions
    (like turn signals and speed limits) using a new PUSH-PULL socket with Unity.

    Args:
        q (Queue): The queue processing the requests
    """
    driving_socket = zmq.Context().socket(zmq.PULL)
    driving_socket.bind("tcp://127.0.0.1:5559")

    while 1:
      rcv = driving_socket.recv().decode("utf-8")
      queue.put(control_cmd_gen(rcv))

  # This is an override of the parent function to add the driving_prms
  def run(self, queue, retries=-1):

    log.info('`run(q)` called. Starting the `bridge_keep_alive` process.')
    bridge_p = Process(name="bridge", target=self.bridge_keep_alive, args=(queue, retries))
    bridge_p.start()

    # Start the get_driving_prms function in a new process
    driving_prms_p = Process(name="driving_prms", target=self.get_driving_prms, args=(queue, ))
    driving_prms_p.start()

    return bridge_p, driving_prms_p