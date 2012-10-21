#!/bin/sh 
set-mote-id build/cricket/main.exe build/cricket/main.exe.out $@
avr-objcopy --output-target=ihex build/cricket/main.exe.out build/cricket/main.hex
