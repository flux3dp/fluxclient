
def check_pcl():
    try:
        from fluxclient.scanner import _scanner
        return True
    except (ImportError, AttributeError):
        return False


def check_platform():
    import platform
    p = platform.platform()
    if p.startswith("Windows"):
        p = "Windows"
    elif p.startswith("Linux"):
        p = "Linux"
    elif p.startswith("Darwin"):
        p = "OSX"
    return (p, platform.architecture()[0])

__version__ = "1.3.5"
SUPPORT_PCL = check_pcl()
