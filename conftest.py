
def _get_tempdir():
    import os
    tmp_folder = os.path.join(
        os.path.dirname(
            os.path.abspath(__file__)), "tmp")
    if not os.path.exists(tmp_folder):
        os.mkdir(tmp_folder)
    return tmp_folder


def _setup_logger(tmp_folder):
    import logging.config
    import os
    logging.config.dictConfig({
        'version': 1,
        'disable_existing_loggers': True,
        'formatters': {
            'default': {
                'format': "[%(name)s %(levelname)s] %(message)s",
                'datefmt': "%Y-%m-%d %H:%M:%S"
            }
        },
        'handlers': {
            'file': {
                'formatter': 'default',
                'class': 'logging.FileHandler',
                'filename': os.path.join(tmp_folder, "test.log")
            }
        },
        'root': {
            'handlers': ['file'],
            'level': 'DEBUG',
            'propagate': True
        }
    })

TMP_FOLDER = _get_tempdir()
_setup_logger(TMP_FOLDER)
