import yaml
import time
import pybullet as pb

from env import Environment
from exoskeleton import ExoSkeleton


class Simulation:
    """
    Class handling the running of the simulation.
    """
    config_path = 'config.yaml'
    config = None

    def __init__(self):
        """
        Initializes dependencies.
        """
        self._load_configuration()
        env = Environment(self.config)
        exo_skel = ExoSkeleton(self.config)

    def _load_configuration(self):
        """
        Loads the yaml file at config_path into a dict.
        """
        with open(self.config_path) as config_file:
            try:
                self.config = yaml.safe_load(config_file)
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
