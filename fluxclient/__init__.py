
def check_pcl():
    try:
        from .scanner import _scanner
        return True
    except (ImportError, AttributeError):
        return False


__version__ = "0.8b12"
SUPPORT_PCL = check_pcl()
