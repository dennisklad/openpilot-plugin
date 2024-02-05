import numpy as np
from mss import mss

from panda3d.core import Texture, GraphicsOutput

from openpilot.tools.sim.bridge.common import SimulatorBridge
from openpilot.tools.sim.bridge.unreal.unreal_world import UnrealWorld
from openpilot.tools.sim.lib.camerad import W, H
from openpilot.tools.sim.lib.common import World

class UnrealBridge(SimulatorBridge):
  TICKS_PER_FRAME = 5

  def __init__(self, args):
    self.should_render = False

    super(UnrealBridge, self).__init__(args)   # <-------  2) Initialise SimulatorBridge
 
  def spawn_world(self) -> World:

    # TODO
    # What comes here?
    # What is the output
    # Start with a still road image?

    
    
    
    return UnrealWorld()

