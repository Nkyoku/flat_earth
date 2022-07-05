# flat_earth
[EarthTfPublisher](#EarthTfPublisher) publishes the TrasnformStamped between the earth frame and the map frame.
The reference point is specified by the GeoPoint message.
The transform between the earth and the map changes continuously by changing the GeoPoint.

# EarthTfPublisher

## Subscribing Topics
- `ref` ([geographic_msgs/GeoPointStamped](https://docs.ros.org/en/noetic/api/geographic_msgs/html/msg/GeoPointStamped.html))  
  The contact point of the map frame and the earth ellipsoid.  
  The first ref topic will be used as origin of the map frame.

## Publishing Topics
- `/tf` ([tf2_msgs/TFMessage](https://docs.ros.org/en/noetic/api/tf2_msgs/html/msg/TFMessage.html))  
  Two transform will be published.
  - `earth` to `ref`
  - `earth` to `map`

## Parameters
- `~earth_frame_id` : str
- `~map_frame_id` : str
- `~ref_frame_id` : str
- `~publish_period` : float
- `~stamp_offset` : float
