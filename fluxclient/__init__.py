
def check_pcl():
    try:
        from .scanner import _scanner
        return True
    except ImportError:
        return False


__version__ = "0.8b1"
SUPPORT_PCL = check_pcl()
