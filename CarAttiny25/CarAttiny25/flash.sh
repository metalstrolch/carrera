#! /bin/bash
avrdude -c usbasp -p t25 -U flash:w:/tmp/CarAttiny25.hex:a
