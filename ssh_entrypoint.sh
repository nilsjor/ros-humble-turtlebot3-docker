#!/bin/bash

# Check if NET_ADMIN capability is available
if /sbin/capsh --has-p='cap_net_admin' ; then

    # Set the nameserver
    if [ -z "${DNS_IP}" ]; then
        echo "Environment variable DNS_IP is not defined. Using Cloudflare as default."
        echo nameserver 1.1.1.1 > /etc/resolv.conf
    else
        echo "Nameserver is defined as ${DNS_IP}."
        echo nameserver $DNS_IP > /etc/resolv.conf
    fi

    # Set the default gateway
    if [ -z "${GATEWAY_IP}" ]; then
        echo "Environment variable GATEWAY_IP is not defined."
    else
        echo "Default gateway is defined as ${GATEWAY_IP}."
        ip route del default
        ip route add default via $GATEWAY_IP
    fi
else
    echo "Error: NET_ADMIN capability is not available. Connectivity may not work properly." >&2
fi

# setup ros2 environment
exec /ros_entrypoint.sh "$@"
