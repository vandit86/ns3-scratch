#!/bin/sh

# create bridges
brctl addbr br-left
brctl addbr br-right

# create TAP devices
tunctl -t tap-left
tunctl -t tap-right

# TAP interfaces up in promisc mode 
ifconfig tap-left 0.0.0.0 promisc up
ifconfig tap-right 0.0.0.0 promisc up

# link tap with brifges
brctl addif br-left tap-left
brctl addif br-right tap-right

# bridge interfaces up 
ifconfig br-left up
ifconfig br-right up

# Start LXC containers 
lxc-start -n mp-left
lxc-start -n mp-right

#show container status 
lxc-ls -f
# show interfaces status 
brctl show
