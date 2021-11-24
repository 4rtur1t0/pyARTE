from configuration import *

__configuration_file_path__ = os.path.join(os.path.dirname(__file__), 'experiments.yml')

CONFIGURATION = Configuration(__configuration_file_path__)