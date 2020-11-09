# Simple example which opens file provided as command line argument
# and displays the first source in a window (using OpenCV)

import cv2
import voltu
import sys
import os

if len(sys.argv) != 2:
	print("%s filename" % os.path.basename(__file__))
	exit(1)

api = voltu.System
api.open(sys.argv[1])
room = api.getRoom(0)

while True:
	if room.waitNextFrame(1000):
		frames = room.getFrame().getImageSet(voltu.Channel.kColour)
		im = frames[0].getHost()
		cv2.imshow("im", im)

	if cv2.waitKey(10) == 27:
		break
