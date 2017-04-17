uint16 object_age
uint16 hidden_status_age
uint8 object_phase
uint8 dynamic_property
float64 relative_time_of_measure
Point position_closest_point
Vector3WithSigmas relative_velocity
uint8 object_class
uint16 classification_age
OrientedBoxWithSigmas object_box
uint8 tracking_point_location
PointWithSigmas tracking_point_coordinate
TwistWithSigmas velocity
AccelWithSigmas acceleration
float64 yaw_rate
float64 yaw_rate_sigma
Polygon[] object_polygon

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
