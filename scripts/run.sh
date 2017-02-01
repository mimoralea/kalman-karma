#!/bin/bash

# Log into the remote machine and run a single command.
# Change the IP address to the address of the EV3.
# Call the command with ./run.sh "test.py args" if you need to pass arguments.

ssh root@192.168.13.102 'python '$1


