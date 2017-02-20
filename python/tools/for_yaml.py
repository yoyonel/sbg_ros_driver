import yaml
from for_logger import *

"""Create and use a logger."""
logger = setup_logger()


def configurations_from_yaml(yaml_setting_filename="settings/settings.yml"):
    """

    :param yaml_setting_filename:
    :return:
    """
    with open(yaml_setting_filename, 'r') as ymlfile:
        logger.info("Load YAML settings file: '%s'" % yaml_setting_filename)
        cfg = yaml.load(ymlfile)
        return cfg


def walk_preorder(yaml_node, prepath=""):
    """

    :param yaml_node:
    :param prepath:
    :return:
    """
    # print("prepath: {}".format(prepath))
    if type(yaml_node) is dict:
        prepath += "/"
        # print("cfg: {}".format(cfg))
        for key, value in yaml_node.iteritems():
            # print("key: {}".format(key))
            for descendant in walk_preorder(value, prepath + key):
                yield descendant
    else:
        yield (prepath, yaml_node)

