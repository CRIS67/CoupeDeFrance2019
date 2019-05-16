#!/bin/bash
echo 'Sending files to RPI ...' 
rsync ./* pi@10.3.141.1:/home/pi/turupo/
