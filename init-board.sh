#!/bin/bash

avrdude -p atmega168 -P usb -c avrispmkII  -U lfuse:w:0xE2:m
avrdude -p atmega168 -P usb -c avrispmkII  -U lfuse:w:0xE6:m
make program
