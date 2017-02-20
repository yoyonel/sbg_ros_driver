#!/usr/bin/python
# vim: set fileencoding=utf-8
import clang.cindex

from for_clang import *
from for_generation import *
from for_logger import *


# import logging


class ClangAST:
    """

    """

    def __init__(self, yaml_setting_filename=""):
        """

        :param yaml_setting_filename:
        """
        # Setup Logging
        setup_logging()
        self.logger = logging.getLogger(__name__)

        # Configurations
        self.cfg = None
        self.clang_cfg = None
        # CLANG
        self.clang_index = None
        self.clang_options = None
        self.clang_cfg_locations = None
        #
        self.source_filename = None

        # Load settings
        self.load_settings(yaml_setting_filename)

    def load_settings(self, settings_filename):
        """

        :param settings_filename:
        :return:
        """
        if settings_filename:
            self.cfg = configurations_from_yaml(settings_filename)

            try:
                ############################################################
                # Récupérations des données de configurations (clang, locations, ...)
                # à partir du fichier de configuration YAML
                ############################################################
                self.clang_cfg = self.cfg.get('clang')
                #
                self.clang_cfg_locations = self.clang_cfg.get('locations')
                # set the path to libclang library
                clang.cindex.Config.set_library_file(self.clang_cfg_locations.get('libclang'))
                self.clang_index = clang.cindex.Index.create()
                # add compiler options
                self.clang_options = self.clang_cfg.get('options')
                # add includes paths
                self.clang_options.extend(
                    ['-I{}'.format(os.path.abspath(include_path))
                     for include_path in self.clang_cfg_locations.get('includes')]
                )
                ############################################################
            except KeyError, e:
                logging.error("KeyError - %s" % str(e))
        else:
            logging.warning("load_settings: no setting_filename given !")

    def process(self):
        """

        :return:
        """
        ################################################################################################################
        # get the path to the first source code to analyze: sbgEComBinaryLogs.h
        ################################################################################################################
        filename = self.clang_cfg_locations['sources']['sbgEComBinaryLogs']

        # parse clang du fichier source
        tu = self.clang_index.parse(filename, self.clang_options)

        ###########################################
        # ROS Messages
        ###########################################
        self.process_ros_msgs(tu)

        ###########################################
        # Association des types des sbglog avec les enums
        ###########################################
        # on recupère une liste de tuples de types (var name, var type, enum/comment)
        bound_check_filename = partial(filter_source_filename, location_filename=filename)
        sblog_types = extract_tuples_sbglog_types(tu, bound_check_filename)

        #
        sbglogs = extract_enums_from_comments(sblog_types)

        sbglogs_filtered = self.process_sbgEComIds(sbglogs)

        # Generate .h .cpp sources files from sbglogs analysis.
        generate_cpp_files(sbglogs_filtered, self.cfg)

    def process_sbgEComIds(self, sbglogs):
        ##########################################################################
        # filtre les comments/enums avec des '#' (SBG_ECOM_LOG_*#)
        # on traite ces cas 'génériques' plus tard ...
        sbglogs_filtered = filter(lambda sbglob: all('#' not in e for e in sbglob.enum), sbglogs)

        ################################################################################################################
        # get the path to the second source code to analyze: sbgEComIds.h
        ################################################################################################################
        filename = self.clang_cfg_locations['sources']['sbgEComIds']

        tu = self.clang_index.parse(filename, self.clang_options)

        bound_check_filename = partial(filter_source_filename, location_filename=filename)

        list_enums = list(n.spelling
                          for n in iter_nodes(tu.cursor.walk_preorder(),
                                              [bound_check_filename, is_enum_constant_decl]))

        sbglogs_enums_contains_sharp = filter(lambda sbglob: not all('#' not in enum for enum in sbglob.enum), sbglogs)
        for sbglog in sbglogs_enums_contains_sharp:
            enums = sbglog.enum
            for enum in enums:
                if '#' in enum:
                    regexp = enum.replace('#', '(.*)')
                    found_enums = list(set(filter(lambda enum: re.search(regexp, enum), list_enums)))
                    self.logger.info("{} -> {}".format(enum, found_enums))
                    #
                    for new_enum in found_enums:
                        new_sbglog = sbglog._replace(enum=[new_enum])
                        # pour etendre la liste des types logs qu'on gère
                        sbglogs_filtered.append(new_sbglog)
        ################################################################################################################
        self.logger.info("\n".join(map(str, sbglogs_filtered)))

        return sbglogs_filtered

    def process_ros_msgs(self, tu):
        """

        :param tu:
        :return:
        """
        # Récupération de la liste des headers
        headers = list(extract_headers_from_ast(tu))

        typedefs_from_headers = extract_typedefs_from_headers(headers, self.clang_index, self.clang_options)

        generate_ros_msg_from_typedefs(typedefs_from_headers, path="gen/msg/")
