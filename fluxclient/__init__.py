
def check_pcl():
    try:
        # from fluxclient.scanner import _scaner
        return True
    except ImportError:
        return False


VERSION = ("0", "5a1")

SUPPORT_PCL = check_pcl()
