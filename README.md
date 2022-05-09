# Using NS3-based Emulation environment  
This page contains an detailed explanation of how-to use our emulation framework, information about useful tools, configuration examples and many more.. 


### ns-3 instalation 
Installation in chroot jail https://www.projectguideline.com/installing-ns3-35-in-debian-10-chroot-jail-under-debian-11-host-os-or-any-version-of-linux-host/ 

## Compilation
### optimized 
> ./waf configure --build-profile=optimized --out=build/optimized --enable-examples --disable-python --enable-tests --enable-sudo --disable-werror  

### debug 
> ./waf configure --build-profile=debug --out=build/debug --enable-examples --disable-python --enable-tests --enable-sudo --disable-werror  

## Common 
### Redirect to file 
> ./waf --run "m_wave_80211p" > log.out 2>&1

### find examples in code 
> find . -name '*.cc' | xargs grep NS_LOG_COMPONENT_DEFINE | grep -i WifiNetDevice
> find . −name ’ ∗ . cc ’ | xargs grep CourseChange | grep Connect

### debuging GDB NS3  
Start with: to load it in gdb:
> ./waf --run "mp-tap-wifi-lte --simTime=60" --gdb

**Quick commands:**
Type "run" to start execution. When it fails, type "bt" to get backtrace on where you were going. Restart, but this time before "run" set breakpoints, e.g. break mysource.cc:1234 to stop execution at that point. Then you can either type n or s to step line-by-line through the execution (s steps into function calls, n does not). You can use 'print varname' to check contents of variables and pretty much anything else (for class variables, you might need to use 'print this->classvar').

## Interact with the real world
different possibilities : https://www.nsnam.org/wiki/HOWTO_make_ns-3_interact_with_the_real_world 

Virtual Machine: https://www.nsnam.org/wiki/HOWTO_use_VMware_to_set_up_virtual_networks_(Windows)

LInux Containers https://www.nsnam.org/wiki/HOWTO_Use_Linux_Containers_to_set_up_virtual_networks

Combine with CORE network emulator: https://www.nsnam.org/wiki/HOWTO_Use_CORE_to_test_ns-3_protocols
	
ns3-lxc project, released on June 17, 2017.	https://github.com/buzz66boy/ns3-lxc/wiki/Release-v0.1

## NS3 Tools

integration with mininet : https://github.com/mininet/mininet/wiki/Link-modeling-using-ns-3 

3d visualizer (new):  https://github.com/usnistgov/NetSimulyzer

Topology gen : https://github.com/idaholab/Topology_Generator

### ns-3 V2X TOOLS 
ns-3 modules to build and simulate ETSI-compliant VANET (V2X) applications using SUMO (v-1.6.0+) and ns-3 (v-3.33), with the possibility of easily switching stack and communication technology. https://github.com/marcomali/ms-van3t

Vehicle simulation program capable of exporting traces in ns2/ns3 format https://github.com/DenisCobeti/Mogen

Co-simulate MATLAB with NS-3 network simulator, combining the powers of MATLAB and NS-3 (Supports modeling WLAN network and 802.11p based V2X scenarios) https://github.com/vkrepo/MATLAB-NS3

Direct Code Execution (DCE) is a framework for ns-3 that provides facilities to execute, within ns-3, existing implementations of userspace and kernelspace network protocols or applications without source code changes.
support only  Ubuntu 16.04 and equivalents (gcc-5.4, libc-2.23)
https://github.com/direct-code-execution/ns-3-dce

# Using LXC
#### **Configuration**
**main link :**

<https://ubuntu.com/server/docs/containers-lxc>

<https://eax.me/lxc/>

**Introduction to linux interfaces for virtual networking**

<https://developers.redhat.com/blog/2018/10/22/introduction-to-linux-interfaces-for-virtual-networking#>

##### ***configure VBox***
**VBox config after patch kernel:** changed the graphics controller to VBoxVGA"

not working with NAT to access internet with mptcp :

*The design of Multipath TCP has been heavily influenced by the middleboxes that have been deployed in a wide range of networks, notably in cellular and enterprise networks. Some of these middleboxes like regular NATs interact correctly with Multipath TCP and many Multipath TCP users work behind NATs. However, some middleboxes, such as firewalls or TCP optimisers, terminate TCP connections or interfere with TCP options and thus interact badly with Multipath TCP.*


working with **Network Bridge** network configuration..

install guest additions :

<https://linuxize.com/post/how-to-install-virtualbox-guest-additions-in-ubuntu/>

##### ***create and configure container***  
find linux container :

<https://us.images.linuxcontainers.org/>

>mint	ulyssa	amd64 -> tested but not working
>mint	sarah	amd64 -> **actually working**

lxc configuration keys

https://linuxcontainers.org/lxc/manpages/man5/lxc.container.conf.5.html

>sudo tunctl -t tap-right
>sudo ifconfig tap-right 0.0.0.0 promisc up

**create linux container** with 2 interfaces (see .conf on scratch)

**sudo lxc-create -t download -n mycontainer -- -d ubuntu -r xenial -a amd64 --keyserver hkp://keyserver.ubuntu.com**

>sudo chroot /var/lib/lxc/mp-left/rootfs/ passwd
>sudo lxc-start -n left


##### ***configure existing LXC containers***  
> sudo nano /var/lib/lxc/right/config
> sudo nano /var/lib/lxc/left/config

##### ***lxc-chekconfig :***
`	`**see required modules : [https://programmersought.com/article/78551456854/**](https://programmersought.com/article/78551456854/)**

Note : Before booting a new kernel, you can check its configuration

usage : CONFIG=/path/to/config /usr/bin/lxc-checkconfig

##### ***lxc-copy :***
copy container lxc-copy -n C1  -NC2

***execute script on startup***

main discussion:  

<https://github.com/lxc/lxc/issues/496>

**crete script for linux containers**, and Make it executable (chmod 755): add hook to config file of container

lxc.hook.mount = /var/lib/lxc/mp-right/ifup.sh
sudo xed /var/lib/lxc/mp-left/ifup.sh

**OR add script to file on container**
sudo nano /etc/rc.local


##### ***Limitations***

**Not every device is able to connect to LXC from NS3 (irrelevant if use FdNetDev)**

*use **promiscuous mode in the bridged device t**o receive packets destined for the different MAC*
*SendFrom() which allows a caller to "spoof" or change the source MAC address to match the different Linux MAC address*
*allows us to use Send() on the ns-3 device side which is available on all ns-3 net devices.*

*Linux host writes to one of the /dev/tap devices,the write is redirected into the TapBridge that lives in the ns-3 world*

*only ns-3 net devices that support SendFrom() and have a hookable promiscuous receive callback are allowed to participate in UseBridge mode TapBridge configurations.*
#### **Routing**
**see route**
>ip route show
>route
>netstat -nr

***add route***
> sudo ip route add 172.16.0.0/24 via 192.168.122.1 dev ens3
> sudo route add -net 10.0.0.0/8 gw 192.168.1.1 eth0
> sudo ip route add default via 192.168.1.254

**add default route  to lxc container configuration file** 

> lxc.net.1.ipv4.gateway = 10.0.1.1

**delete route**
sudo route del -net 192.168.3.0 gw 192.168.1.1 netmask 255.255.255.0 dev eth0



