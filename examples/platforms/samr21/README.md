# OpenThread SAMR21 Example

This directory contains example platform drivers for the [ATMEL SAMR21][samr21]

**NOTE:** Each Thread node requires a unique EUI-64.  
Please make sure all Thread nodes in your network have a unique EUI-64 by setting HARDCODED_NODE_ID in radio.c to a unique value.

## Build Examples (How to build and flash):
```bash
$ cd <path-to-openthread>
$ ./bootstrap
$ make -f examples/Makefile-samr21 clean
$ make -f examples/Makefile-samr21
```

Execute script to prepare image for flashing:

**NOTE:** Flashing process requires JLinkGDBServer. Please visit https://www.segger.com/jlink-gdb-server.html for details.

1. Open another terminal window in order to start gdb server. Write the command (Linux):

   ```bash
   $ JLinkGDBServer -ir -if swd -speed auto -device Cortex-M0+ -logtofile on -localhostonly
   ```

## Interact:

Board will indicate state of device according to LED blink speed.

| Frequency | Role |
| --- | --- |
| 5Hz | Leader |
| 2Hz | Router |
| 0.5Hz | End Device |


1. Open terminal to /dev/ttyACM0 (serial port settings: 9600 8-N-1).
2. Type help for list of commands

## Examples:
1. Create Topology and become Leader on channel 13.
   ```
   > channel 13
   Done
   > panid 0xface
   Done
   > ifconfig up
   Done
   > thread start
   Done
   ```
   Wait and see that LED started to blink with 5Hz.
   
   Check node state:
   ```
   > state
   Leader
   ```
2. Attach Router and ping Leader Mesh Local address.

   Open terminal for second device /dev/ttyACM1 and issue commands:
   ```
   > channel 13
   Done
   > panid 0xface
   Done
   > ifconfig up
   Done
   > thread start
   ```

   Wait until LED start to blink. Should blink slower.

   Check node state:

   ```
   > state
   Router
   ```

   List addresses on terminal with Leader:

   ```  
   > ipaddr
   fdde:ad00:beef:0:0:ff:fe00:4400
   fdde:ad00:beef:0:92f5:9844:67ad:a0c9
   fe80:0:0:0:2c11:aba1:ecc1:96da
   ```

   Ping Leader from Router terminal:

   ```
   > ping fdde:ad00:beef:0:92f5:9844:67ad:a0c9
   16 bytes from fdde:ad00:beef:0:92f5:9844:67ad:a0c9: icmp_seq=2 hlim=64 time=359ms
   ```
 
