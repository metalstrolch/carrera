#! /bin/bash

${TARGET_CROSS}avr-gcc -mmcu=attiny25 -DF_CPU=8000000UL -Os -o /tmp/CarAttiny25 main.c
${TARGET_CROSS}avr-objcopy -j .text -j .data -O ihex /tmp/CarAttiny25 /tmp/CarAttiny25.hex
