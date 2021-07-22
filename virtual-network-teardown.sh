#!/bin/sh

#stop containers
lxc-stop -n mp-left -k
lxc-stop -n mp-right -k

#lxc-destroy -n left
#lxc-destroy -n right

# bridge infaces down 
ifconfig br-left down
ifconfig br-left-1 down
ifconfig br-right down
ifconfig br-right-1 down

# delete tap-bridge connections 
brctl delif br-left tap-left
brctl delif br-left-1 tap-left-1
brctl delif br-right tap-right
brctl delif br-right-1 tap-right-1

# delete bridges 
brctl delbr br-left
brctl delbr br-left-1
brctl delbr br-right
brctl delbr br-right-1

# TAP infces down 
ifconfig tap-left down
ifconfig tap-left-1 down
ifconfig tap-right down
ifconfig tap-right-1 down

# delete TAP's infaces
tunctl -d tap-left
tunctl -d tap-left-1
tunctl -d tap-right
tunctl -d tap-right-1
