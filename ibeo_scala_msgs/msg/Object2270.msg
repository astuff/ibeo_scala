Header header

uint16 object_id
uint16 object_age
uint16 predection_age
uint16 relative_moment_of_measurement
uint8 reference_point_location
PointWithSigma reference_point
Point contour_points_cog
SolidPrimitive object_box
float64 object_box_orientation_angle
float64 object_box_orientation_angle_sigma
Vector3WithSigma absolute_velocity
Vector3WithSigma relative_velocity
uint8 classification
uint8 tracking_model
bool mobile_detected
bool track_valid
uint16 classification_age
uint16 classification_confidence
Point[] contour_points

# reference_point_location values
uint8 CENTER_OF_GRAVITY=0
uint8 TOP_FRONT_LEFT_CORNER=1
uint8 TOP_FRONT_RIGHT_CORNER=2
uint8 BOTTOM_REAR_RIGHT_CORNER=3
uint8 BOTTOM_REAR_LEFT_CORNER=4
uint8 CENTER_OF_TOP_FRONT_EDGE=5
uint8 CENTER_OF_RIGHT_EDGE=6
uint8 CENTER_OF_BOTTOM_REAR_EDGE=7
uint8 CENTER_OF_LEFT_EDGE=8
uint8 BOX_CENTER=9
uint8 INVALID=255

# classification values
uint8 UNCLASSIFIED=0
uint8 UNKNOWN_SMALL=1
uint8 UNKNOWN_BIG=2
uint8 PEDESTRIAN=3
uint8 BIKE=4
uint8 CAR=5
uint8 TRUCK=6
uint8 OVER_DRIVABLE=10
uint8 UNDER_DRIVABLE=11

# tracking_model values
uint8 DYNAMIC_MODEL=0
uint8 STATIC_MODEL=1
