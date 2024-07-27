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

    if [ -n "$L2TP_BROADCAST" ]; then
        ip addr add ${L2TP_ENDPOINT_CIDR} broadcast ${L2TP_BROADCAST} dev l2tp0
    else
        ip addr add ${L2TP_ENDPOINT_CIDR} dev l2tp0
    fi

else
    echo "Error: NET_ADMIN capability is not available. L2TP tunnel has not been configured!" >&2
fi

# setup ros2 environment
exec /ros_entrypoint.sh "$@"
