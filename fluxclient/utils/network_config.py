

def parse_network_config(method, wifi_mode=None, ipaddr=None, mask=None,
                         route=None, ns=None, ssid=None, scan_ssid=None,
                         security=None, wepkey=None, psk=None):
    assert method in ["dhcp", "static"], "method must be dhcp or static"

    options = {"method": method}
    if method == "static":
        assert ipaddr, "ipaddr is required"
        assert mask and isinstance(mask, int), "mask is required"
        assert route, "route is required"
        assert ns, "ns is required"

        options.update({
            "ipaddr": ipaddr, "mask": mask, "route": route, "ns": ns,
        })

    if ssid:
        options["wifi_mode"] = wifi_mode
        options["ssid"] = ssid
        if scan_ssid:
            options["scan_ssid"] = "1"

        if security == "WEP":
            assert wepkey, "wepkey is required"
            options["security"] = security
            options["wepkey"] = wepkey
        elif security in ['WPA-PSK', 'WPA2-PSK']:
            options["security"] = security
            options["psk"] = psk

    return options
