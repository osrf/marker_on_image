# marker_on_image

Subscribes to camera images and 3D markers and republishes the image with markers overlayed.

Run:

    rosrun marker_on_image marker_on_image_node _in_marker_topic:="/some_markers" _in_image_topic:="/some/camera" _out_image_topic:="/some_image"

TODO:

* Support `LINE_STRIP` and `LINE_LIST`
* Support deletion
* (Publish transparent image with only the overlays)
