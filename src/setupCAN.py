#!/usr/bin/env python

import os

if __name__ == '__main__':

	bashCommand = "sudo /sbin/ip link set can0 up type can bitrate 400000"
	os.system(bashCommand)