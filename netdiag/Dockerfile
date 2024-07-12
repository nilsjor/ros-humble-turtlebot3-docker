ARG BASE_IMAGE
FROM ${BASE_IMAGE} AS base

RUN apt-get update && apt-get install -y \
    iputils* \
    net-tools \
    iproute2 \
    tcpdump \
    traceroute \
    mtr-tiny \
    dnsutils \
    nmap \
    curl \
    wget
