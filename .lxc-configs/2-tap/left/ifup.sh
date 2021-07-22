#!/bin/sh
#ip route add 13.0.0.3/32 via 11.0.0.1 dev eth1
#ip route add 13.0.0.0/8 via 11.0.0.1 dev eth0
ip route add default via 11.0.0.1
ip route add 14.0.0.0/8 via 15.0.0.1 dev eth1

