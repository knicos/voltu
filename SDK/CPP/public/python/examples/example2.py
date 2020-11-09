# Simple example which opens file provided as command line argument
# and renders a virtual camera into window (using OpenCV)

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
cam = api.createCamera()

while True:
	if room.waitNextFrame(1000):
		cam.submit(room.getFrame())
		frames = cam.getFrame().getImageSet(voltu.Channel.kColour)
		im = frames[0].getHost()
		cv2.imshow("im", im)

	if cv2.waitKey(10) == 27:
		break
