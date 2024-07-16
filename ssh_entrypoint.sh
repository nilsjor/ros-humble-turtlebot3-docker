#!/bin/bash

# Check if NET_ADMIN capability is available
if /sbin/capsh --has-p='cap_net_admin' ; then

    # Set the nameserver
    echo "Nameserver set to ${DNS_IP}."
    echo nameserver $DNS_IP > /etc/resolv.conf

    # Set the default gateway
    echo "Default gateway set to ${GATEWAY_IP}."
    ip route del default
    ip route add default via $GATEWAY_IP

else
    echo "Error: NET_ADMIN capability is not available. Outbound connection may not work properly." >&2
fi

# Set root password
if [ -n "$PASS" ]; then
    echo "root:$PASS" | chpasswd
    echo "Root password set from PASS environment variable."
else
    echo "Warning: Environment variable PASS is not defined. Using empty password."
    echo "root:" | chpasswd -e
fi

# Start the SSH daemon
service ssh start

# setup ros2 environment
exec /l2tp_entrypoint.sh "$@"
