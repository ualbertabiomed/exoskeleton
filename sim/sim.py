import yaml
import time
import pybullet as pb

from env import Environment
from exoskeleton import ExoSkeleton

CONFIG_PATH = 'config.yaml'


class Simulation:
    """
    Class handling the running of the simulation.
    """
    def __init__(self):
        """
        Initializes dependencies.
        """
        self._config = self._load_configuration()
        self._env = Environment(self._config)
        self._exo_skel = ExoSkeleton(self._config)

    def _load_configuration(self):
        """
        Loads the yaml file at CONFIG_PATH into a dict.

        :returns: dict containing configuration data in the yaml file at CONFIG_PATH
        """
        with open(CONFIG_PATH) as config_file:
            try:
                return yaml.safe_load(config_file)
            except yaml.YAMLError as e:
                print('YAMLError: ' + str(e))
                exit()

    def run(self):
        """
        Runs the simulation.
        """
        # TODO control robot, etc. here
        for _ in range(1000):
            pb.stepSimulation()
            time.sleep(1. / 240.)


if __name__ == '__main__':
    Simulation().run()
