
def check_pcl():
    try:
        # from fluxclient.scanner import _scaner
        return True
    except ImportError:
        return False


VERSION = ("0", "6a2")
__version__ = ".".join(VERSION)
SUPPORT_PCL = check_pcl()
