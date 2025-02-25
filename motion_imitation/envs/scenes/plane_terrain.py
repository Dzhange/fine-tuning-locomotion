# Original script: https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/heightfield.py
import pybullet as p
import random

FLAG_TO_FILENAME = {
    'mounts': "heightmaps/wm_height_out.png",
    'maze': "heightmaps/Maze.png"
}

ROBOT_INIT_POSITION = {
    'mounts': [0, 0, .85],
    'plane': [0, 0, 0.21],
    'hills': [0, 0, 1.98],
    'maze': [0, 0, 0.21],
    'random': [0, 0, 0.21]
}


class Terrain:

    def __init__(self, terrain_source, terrain_id, columns=256, rows=256):
        random.seed(10)
        self.terrain_source = terrain_source
        self.terrain_id = terrain_id
        self.columns = columns
        self.rows = rows

    def generate_terrain(self, env, height_perturbation_range=0.05):
        env.pybullet_client.setAdditionalSearchPath(pd.getDataPath())
        env.pybullet_client.configureDebugVisualizer(env.pybullet_client.COV_ENABLE_RENDERING, 0)
        height_perturbation_range = height_perturbation_range
        terrain_data = [0] * self.columns * self.rows
        if self.terrain_source == 'random':
            for j in range(int(self.columns / 2)):
                for i in range(int(self.rows / 2)):
                    height = random.uniform(0, height_perturbation_range)
                    terrain_data[2 * i + 2 * j * self.rows] = height
                    terrain_data[2 * i + 1 + 2 * j * self.rows] = height
                    terrain_data[2 * i + (2 * j + 1) * self.rows] = height
                    terrain_data[2 * i + 1 + (2 * j + 1) * self.rows] = height
            terrain_shape = env.pybullet_client.createCollisionShape(
                shapeType=env.pybullet_client.GEOM_HEIGHTFIELD,
                meshScale=[.05, .05, 1],
                heightfieldTextureScaling=(self.rows - 1) / 2,
                heightfieldData=terrain_data,
                numHeightfieldRows=self.rows,
                numHeightfieldColumns=self.columns)
            terrain = env.pybullet_client.createMultiBody(0, terrain_shape)
            env.pybullet_client.resetBasePositionAndOrientation(terrain, [0, 0, 0], [0, 0, 0, 1])


        self.terrain_shape = terrain_shape
        env.pybullet_client.changeVisualShape(terrain, -1, rgbaColor=[1, 1, 1, 1])
        # env.pybullet_client.configureDebugVisualizer(env.pybullet_client.COV_ENABLE_RENDERING, 1)