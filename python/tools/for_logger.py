# import logging
import logging.config
import os

import yaml
from colorlog import ColoredFormatter


def setup_logger(name=""):
    """

    :param name:
    :return:
    """
    """Return a logger with a default ColoredFormatter."""
    # %(white)s%(asctime)s%(reset)s
    formatter = ColoredFormatter(
        "%(log_color)s%(levelname)-8s%(reset)s %(blue)s%(message)s",
        datefmt=None,
        reset=True,
        log_colors={
            'DEBUG': 'cyan',
            'INFO': 'red',
            'WARNING': 'yellow',
            'ERROR': 'red',
            'CRITICAL': 'red',
        }
    )

    logger = logging.getLogger(name)
    handler = logging.StreamHandler()
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    logger.setLevel(logging.DEBUG)

    return logger


def test_logger(logger):
    """

    :param logger:
    :return:
    """
    logger.debug('a debug message')
    logger.info('an info message')
    logger.warning('a warning message')
    logger.error('an error message')
    logger.critical('a critical message')


# """Create and use a logger."""
# logger = setup_logger()

def setup_logging(
        default_path='logging.yaml',
        default_level=logging.INFO,
        env_key='LOG_CFG'
):
    """Setup logging configuration

    :param default_path:
    :param default_level:
    :param env_key:
    :param formatter:
    :return:
    """
    value = os.getenv(env_key, None)
    path_toconfig = value if value else default_path

    path_toconfig = path(path_toconfig)

    if os.path.exists(path_toconfig):
        with open(path_toconfig, 'rt') as f:
            config = yaml.safe_load(f.read())
        logging.config.dictConfig(config)
        logging.info("Init logging from config file: '%s'" % path_toconfig)
    else:
        logging.basicConfig(level=default_level)
        logging.info("Init logging from basicConfig(level=%s)" % default_level)


def path(filename):
    """Return an absolute path to a file in the current directory."""
    return os.path.join(os.path.dirname(os.path.realpath(__file__)), filename)


def setup_logging_fromfile(config_filename="logging.ini"):
    """

    :param config_filename:
    :return:
    """
    logging.config.fileConfig(path(config_filename))
