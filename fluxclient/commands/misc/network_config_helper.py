
from ipaddress import IPv4Interface, IPv4Address


def run():
    from fluxclient.utils.network_config import parse_network_config

    kw = {}
    ret = input("Wifi Mode (0=host,1=client)[1]: ").strip("\n")
    if ret == "0":
        kw["wifi_mode"] = "host"
    else:
        kw["wifi_mode"] = "client"

    kw["ssid"] = input("SSID: ").strip("\n")

    ret = input("Is SSID hidden? (y/n)[n]: ").strip("\n")
    if ret.upper() == "Y":
        kw["scan_ssid"] = "1"

    ret = input("Wifi-Protection (0=None,1=WEP,2=WPA2-PSK)[2]: ").strip("\n")
    if ret == "0":
        kw["security"] = "None"
    elif ret == "1":
        kw["security"] = "WEP"
        kw["wepkey"] = input("Wifi password: ").strip("\n")
    else:
        kw["security"] = "WPA2-PSK"
        kw["psk"] = input("Wifi password: ").strip("\n")

    ret = input("Use DHCP? (0=YES,1=NO)[0]: ").strip("\n")
    if ret == "1":
        kw["method"] = "static"
        ipaddr = IPv4Interface(
            input("IP Address (example: \"192.168.0.1/24\"): ").strip("\n"))
        kw["ipaddr"] = str(ipaddr.ip)
        kw["mask"] = int(ipaddr.with_prefixlen.split("/")[-1])
        gw = IPv4Address(input("Gateway: ").strip("\n"))
        kw["route"] = str(gw)
        kw["ns"] = input("DNS: ").strip("\n")
    else:
        kw["method"] = "dhcp"

    print(">>", kw)
    return parse_network_config(**kw)
