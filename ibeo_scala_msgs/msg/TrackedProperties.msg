uint16 object_age
uint16 hidden_status_age
uint8 object_phase
uint8 dynamic_property
uint16 relative_time_of_measure
Point2Di position_closest_point
Point2Di relative_velocity
Point2Dui relative_velocity_sigma
uint8 object_class
uint16 classification_age
Point2Di object_box_size
Point2Dui object_box_size_sigma
int16 object_box_orientation
uint16 object_box_orientation_sigma
uint8 tracking_point_location
Point2Di tracking_point_coordinate
Point2Dui tracking_point_coordinate_sigma
Point2Di velocity
Point2Dui velocity_sigma
Point2Di acceleration
Point2Dui acceleration_sigma
int16 yaw_rate
uint16 yaw_rate_sigma
ContourPointISigma[] outline_points

# object_phase values
uint8 INITIALIZATION_PHASE=0
uint8 TRACKING_PHASE=1

# dynamic_property values
uint8 DYNAMIC_AND_MOVING=0
uint8 DYNAMIC_AND_STOPPED=1
uint8 A_PRIORI_STATIONARY=2

# object_class values
uint8 UNCLASSIFIED=0
uint8 UNKNOWN_SMALL=1
uint8 UNKNOWN_BIG=2
uint8 PEDESTRIANS=3
uint8 TWO_WHEELER=4
uint8 CAR=5
uint8 TRUCK=6
uint8 OVER_DRIVABLE=10
uint8 UNDER_DRIVABLE=11

# tracking_point_location values
uint8 CENTER_OF_GRAVITY=0
uint8 FRONT_LEFT=1
uint8 FRONT_RIGHT=2
uint8 REAR_RIGHT=3
uint8 REAR_LEFT=4
uint8 FRONT_CENTER=5
uint8 RIGHT_CENTER=6
uint8 REAR_CENTER=7
uint8 LEFT_CENTER=8
uint8 OBJECT_CENTER=9
uint8 UNKNOWN=255
