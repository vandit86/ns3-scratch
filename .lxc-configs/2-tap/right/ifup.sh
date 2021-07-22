#!/bin/sh
#echo "ip route add 15.0.0.0/8 via 14.0.0.1 dev eth1"
#ip route add 11.0.0.3/32 via 13.0.0.1 dev eth1
#ip route add 11.0.0.0/8 via 13.0.0.1 dev eth0
ip route add default via 13.0.0.1
ip route add 15.0.0.0/8 via 14.0.0.1 dev eth1

