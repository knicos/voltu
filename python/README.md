Python support for `.ftl` files. At the moment, only reading RGB channels
(left/right) supported. Non-parallel decoding of 8 streams has a frame rate
of ~15 fps on i7-9700K.

Required **Python** modules:

 * msgpack 
 * numpy
 * skimage **or** OpenCV 

Required libraries

 * libde265 (available on most Linux distributions)
