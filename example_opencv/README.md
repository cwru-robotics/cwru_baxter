start of a package for OpenCV examples

includes a demo (from on-line example) that inputs a stream, superimposes a graphic, and republishes

another that subscribes to an image channel and (via a service), takes snapshots that it
saves to disk as *.png images

another routine, images_typesync.cpp, subscribes to 2 image topics, changes the 
timestamps in their headers to be current and identical to each other, and
republishes these one new topics.  Possibly useful for re-syncing for stereo processing.
