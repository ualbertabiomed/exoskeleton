import pybullet_data
import math
import pybullet as pb


class ExoSkeleton:
    """
    Class representing the exoskeleton.
    """
    def __init__(self, config):
        """
        Initializes the exoskeleton and the human it encloses.

        :param config: dict containing configuration data pertaining to the environment and exoskeleton
        """
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        self._human = pb.loadURDF(
            'humanoid/humanoid.urdf',
            basePosition=config['human']['start-position'],
            baseOrientation=pb.getQuaternionFromEuler(
                list(map(lambda x: math.radians(x), config['human']['start-orientation']))
            ),
            globalScaling=0.25
        )
        # TODO load exoskeleton
