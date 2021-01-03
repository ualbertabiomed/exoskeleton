import pybullet_data
import pybullet as pb


class Environment:
    """
    Class representing a configurable pybullet environment.
    """
    client = None

    def __init__(self, config):
        """
        Initializes the environment.

        :param config: dict containing configuration data pertaining to the environment and exoskeleton
        """
        self._config = config
        self.client = pb.connect(pb.DIRECT) if config['env']['headless'] else pb.connect(pb.GUI)
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        self._set_physics_params()
        self._plane = pb.loadURDF('plane.urdf')

    def _set_physics_params(self):
        """
        Sets physics parameters.
        """
        pb.setGravity(0, 0, -9.81)
        if self._config['env']['real-time']:
            pb.setRealTimeSimulation(1)
