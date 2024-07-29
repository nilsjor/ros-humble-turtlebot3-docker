#!/bin/bash

# Check if NET_ADMIN capability is available
if /sbin/capsh --has-p='cap_net_admin' ; then

    if [ -n "$DNS_IP" ]; then
        # Set the nameserver
        echo "Nameserver set to ${DNS_IP}."
        echo nameserver $DNS_IP > /etc/resolv.conf
    fi

    if [ -n "$GATEWAY_IP" ]; then
        # Set the default gateway
        echo "Default gateway set to ${GATEWAY_IP}."
        ip route del default
        ip route add default via $GATEWAY_IP
    fi
    
else
    echo "Error: NET_ADMIN capability is not available. Outbound connection may not work properly." >&2
fi

# Start the SSH daemon
service ssh start

# setup ros2 environment
exec /ros_entrypoint.sh "$@"
