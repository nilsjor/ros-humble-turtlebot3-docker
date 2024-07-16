#!/bin/bash

# Check if NET_ADMIN capability is available
if /sbin/capsh --has-p='cap_net_admin' ; then

    ip l2tp add tunnel \
        tunnel_id 1 peer_tunnel_id 1 \
        local ${L2TP_LOCAL_IP} remote ${L2TP_REMOTE_IP} \
        encap udp udp_sport ${L2TP_PORT} udp_dport ${L2TP_PORT}

    ip l2tp add session \
        tunnel_id 1 peer_tunnel_id 1 name l2tp0 \
        session_id 1 peer_session_id 1 seq both

    ip link set dev l2tp0 up

    ip addr add dev l2tp0 ${L2TP_ENDPOINT_CIDR}

else
    echo "Error: NET_ADMIN capability is not available. Tunneling may not work properly." >&2
fi

# setup ros2 environment
exec /ros_entrypoint.sh "$@"
