
def check_pcl():
    try:
        # from fluxclient.scanner import _scaner
        return True
    except ImportError:
        return False


VERSION = ("0", "7a1")
__version__ = ".".join(VERSION)
SUPPORT_PCL = check_pcl()
