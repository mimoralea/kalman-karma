#!/bin/bash

# Script to copy a file from the local machine to a remote machine.
# Change the IP address to the address of the EV3.

scp $1 root@192.168.13.102:/root/$1
