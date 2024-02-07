from openpilot.tools.sim.bridge.common import SimulatorBridge
from openpilot.tools.sim.bridge.unreal.unreal_world import UnrealWorld
from openpilot.tools.sim.lib.camerad import W, H
from openpilot.tools.sim.lib.common import World

import logging
log = logging.getLogger('a')

class UnrealBridge(SimulatorBridge):
  TICKS_PER_FRAME = 5

  def __init__(self, args):
    self.should_render = False

    super(UnrealBridge, self).__init__(args)   # <-------  2) Initialise SimulatorBridge
 
  def spawn_world(self) -> World:
    """This function is called once when initiating the bridge.
    The call is found in the _run function of /tools/sim/bridge/common.py

    Returns:
        World: 
    """

    # TODO
    # What comes here?
    # What is the output
    # Start with a still road image?

    log.debug("`UnrealBridge.spawn_world` called. Generating empty UnrealWorld.")

    return UnrealWorld()

