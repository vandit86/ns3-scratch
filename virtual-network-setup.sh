#!/bin/sh

# create bridges
brctl addbr br-left
brctl addbr br-left-1
brctl addbr br-right
brctl addbr br-right-1

# create TAP devices
tunctl -t tap-left
tunctl -t tap-left-1
tunctl -t tap-right
tunctl -t tap-right-1

# TAP interfaces up in promisc mode 
ifconfig tap-left 0.0.0.0 promisc up
ifconfig tap-left-1 0.0.0.0 promisc up
ifconfig tap-right 0.0.0.0 promisc up
ifconfig tap-right-1 0.0.0.0 promisc up

# link tap with brifges
brctl addif br-left tap-left
brctl addif br-left-1 tap-left-1
brctl addif br-right tap-right
brctl addif br-right-1 tap-right-1

# bridge interfaces up 
ifconfig br-left up
ifconfig br-left-1 up
ifconfig br-right up
ifconfig br-right-1 up

# Start LXC containers 
lxc-start -n mp-left
lxc-start -n mp-right

#show container status 
lxc-ls -f
# show interfaces status 
brctl show
