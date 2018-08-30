# marker_on_image

Subscribes to camera images and 3D markers and republishes the image with markers overlayed.

Run:

    rosrun marker_on_image marker_on_image_node _in_marker_topic:="/some_markers" _in_image_topic:="/some/camera" _out_image_topic:="/some_image" _alpha:=0.3

## Supported

Types:

* `CUBE`
* `SPHERE`
* `LINE_STRIP`
* `LINE_LIST`

Actions:

* `ADD`

## Parameters

| Parameter | Type | Default | Description |
| --------- | ---- | ------- | ----------- |
| `in_marker_topic` | string | `markers` | Topic to receive markers (`visualization_msgs/MarkerArray`) |
| `in_image_topic` | string | `camera/rgb/image_raw` | Topic to receive camera images (`sensor_msgs/Image` and `sensor_msgs/CameraInfo`) |
| `out_image_topic` | string | `camera/rgb/image_markers` | Topic to publish images with overlayed markers (`sensor_msgs/Image`) |
| `alpha` | double | -1 | Alpha to be applied to all markers. If not set (< 0), the alpha from each marker message will be used. |

## TODO

* Support deletion
* (Publish transparent image with only the overlays)
