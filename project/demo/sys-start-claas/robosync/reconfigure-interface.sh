#!/bin/bash

IFACE="eth0"
if [[ -z $1 ]]; then
    IFACE=eth0
else
    IFACE=$1
fi
echo "Reconfigureing Interface ${IFACE}"

echo "DHCP Request to get IP"
sudo dhclient -timeout 10 -v ${IFACE}
echo "Bringing IF down and up again"
sudo ifdown ${IFACE}
sudo ifup ${IFACE}
sleep 2
echo "Hopefully reconfigured interface ;)"
echo "If connected to Techfak, dhcp might need up to 2 minutes to get new IP"
