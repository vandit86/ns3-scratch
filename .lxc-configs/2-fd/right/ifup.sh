#!/bin/sh
#echo "ip route add 15.0.0.0/8 via 14.0.0.1 dev eth1"
ip route add 15.0.0.0/8 via 14.0.0.1 dev eth1
ip route add 11.0.0.0/8 via 13.0.0.1 dev eth0

# do not need line below , just to ping test  
ip route add 16.0.0.0/8 via 14.0.0.1 dev eth1
