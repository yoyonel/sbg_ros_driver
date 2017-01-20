import logging
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
            'DEBUG':    'cyan',
            'INFO':     'green',
            'WARNING':  'yellow',
            'ERROR':    'red',
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
