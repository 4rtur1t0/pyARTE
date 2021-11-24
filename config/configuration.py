# -*- coding: utf-8 -*-
"""
.. module:: experiments configuration configuration
   :synopsis: loads a set of experiments to be performed.

.. moduleauthor:: Arturo Gil
"""
import os
from yaml import safe_load

PARAMETERS_YAML = os.path.dirname(__file__) + "experiments.yml"


class Configuration(dict):
    def __init__(self, configuration_file_path=PARAMETERS_YAML, **kwargs):
        super(Configuration, self).__init__(**kwargs)

        self.configuration_file_path = configuration_file_path
        if os.path.exists(self.configuration_file_path):
            loaded_file_obj = safe_load(open(self.configuration_file_path))
        else:
            loaded_file_obj = {}

        self.configurations = loaded_file_obj
