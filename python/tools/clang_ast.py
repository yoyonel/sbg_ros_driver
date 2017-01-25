#!/usr/bin/python
# vim: set fileencoding=utf-8
from for_yaml import configurations_from_yaml
import clang.cindex
import os
from for_logger import *
# import logging


class ClangAST:
    """

    """
    def __init__(self, **kwargs):
        """

        :param yaml_setting_filename:
        """
        # Setup Logging
        setup_logging()
        self.logger = logging.getLogger(__name__)

        yaml_setting_filename = kwargs.get("yaml_setting_filename", None)
        if yaml_setting_filename:
            self.yaml_cfg = configurations_from_yaml(yaml_setting_filename)

            try:
                ############################################################
                # Récupérations des données de configurations (clang, locations, ...)
                # à partir du fichier de configuration YAML
                ############################################################
                self.cfg_clang = self.yaml_cfg.get('clang')
                #
                cfg_clang_locations = self.cfg_clang.get('locations')
                # set the path to libclang library
                clang.cindex.Config.set_library_file(cfg_clang_locations.get('libclang'))
                self.index = clang.cindex.Index.create()
                # add compiler options
                list_options_for_clang = self.cfg_clang.get('options')
                # add includes paths
                list_options_for_clang.extend(
                    ['-I{}'.format(os.path.abspath(include_path))
                     for include_path in cfg_clang_locations.get('includes')]
                )
                ############################################################
            except KeyError, e:
                logging.error("KeyError - %s".format(str(e)))
