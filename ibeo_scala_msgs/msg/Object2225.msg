uint16 object_id
uint32 object_age
time stamp
uint16 hidden_status_age
uint8 classification
uint8 classification_certainty
uint32 classification_age
Point2Di bounding_box_center
Point2Di bounding_box_size
Point2Di object_box_center
Point2Di object_box_center_sigma
float32 yaw_angle
Point2Di relative_velocity
Point2Di relative_velocity_sigma
Point2Di absolute_velocity
Point2Di absolute_velocity_sigma
uint8 closest_point_index
Point2Di[] contour_points

# classification values
uint8 UNCLASSIFIED=0
uint8 UNKNOWN_SMALL=1
uint8 UNKNOWN_BIG=2
uint8 PEDESTRIAN=3
uint8 BIKE=4
uint8 CAR=5
uint8 TRUCK=6
