# Lint as: python3
"""Scene with randomly spaced stepstones."""
import enum
from typing import Any, Dict, List, Optional, Sequence, Text
import numpy as np
import random
from motion_imitation.envs.scenes import stepstones
from pybullet_utils import bullet_client


class ObjectType(enum.Enum):
  """Categories of objects that may be found in a scene."""
  OTHER = 0
  GROUND = 1
  OBSTACLE = 2

class RandomHeightFieldTerrain(object):
  """Scene with randomly spaced stepstones."""
  # start with lower and easy
  def __init__(
      self,      
      num_rows: Optional[int] = 1024,
      num_columns: Optional[int] = 128,
      height_range: Optional[float] = 0.03,
      random_seed: Optional[int] = 10,
      color_sequence: Sequence[Sequence[float]] = stepstones.MULTICOLOR,
      rebuild_scene_during_reset: bool = True):
    """Initializes RandomStepstoneScene.

    Args:
      random_seed: The random seed to generate the random stepstones.
      color_sequence: A list of (red, green, blue, alpha) colors where each
        element is in [0, 1] and alpha is transparency. The stepstones will
        cycle through these colors.
      rebuild_scene_during_reset: Whether to rebuild the stepstones during
        reset.
    """    
    self._num_rows = num_rows
    self._num_columns = num_columns
    self._height_range = height_range
    self._random_seed = random_seed
    self._color_sequence = color_sequence
    self._rebuild_scene_during_reset = rebuild_scene_during_reset

  def reset(self, env):    
    if self._rebuild_scene_during_reset:      
      self.build_scene(self._pybullet_client)
    env.set_ground(self.floor_id)

  def build_scene(self, pybullet_client):
    self._pybullet_client = pybullet_client
      
    random.seed(self._random_seed)
    self._pybullet_client.configureDebugVisualizer(self._pybullet_client.COV_ENABLE_RENDERING,0)    
    heightfieldData = [0]*self._num_rows*self._num_columns
    for j in range (int(self._num_columns/2)):
        for i in range (int(self._num_rows/2) ):
            height = random.uniform(0,self._height_range*5)
            heightfieldData[2*i+2*j*self._num_rows]=height
            heightfieldData[2*i+1+2*j*self._num_rows]=height
            heightfieldData[2*i+(2*j+1)*self._num_rows]=height
            heightfieldData[2*i+1+(2*j+1)*self._num_rows]=height  

    # Add floor 
    terrain_shape = self._pybullet_client.createCollisionShape(
        shapeType = self._pybullet_client.GEOM_HEIGHTFIELD, 
        meshScale=[.02,.02, 1], 
        heightfieldTextureScaling=(self._num_rows-1)/2,
        heightfieldData=heightfieldData,
        numHeightfieldRows=self._num_rows,
        numHeightfieldColumns=self._num_columns)

    terrain_id = self._pybullet_client.createMultiBody(0, terrain_shape)
    self._pybullet_client.resetBasePositionAndOrientation(terrain_id, [0,0,0], [0,0,0,1])
    self.floor_id = terrain_id

  @property
  def pybullet_client(self) -> bullet_client.BulletClient:
    if self._pybullet_client is None:
      raise ValueError("pybullet_client is None; did you call build_scene()?")
    return self._pybullet_client

  @property
  def ground_height(self) -> float:
    """Returns ground height of the scene."""
    return 0.0
  
