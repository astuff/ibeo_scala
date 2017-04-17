Header header

uint16 object_id
uint32 object_age
time stamp
uint16 hidden_status_age
uint8 classification
uint8 classification_certainty
uint32 classification_age
Pose2D bounding_box_center
SolidPrimitive bounding_box
PointWithSigma object_box_center
SolidPrimitive object_box
float32 yaw_angle
Vector3WithSigma relative_velocity
Vector3WithSigma absolute_velocity
uint8 closest_point_index
Point[] contour_points

# classification values
uint8 UNCLASSIFIER=0
uint8 UNKNOWN_SMALL=1
uint8 UNKNOWN_BIG=2
uint8 PEDESTRIAN=3
uint8 BIKE=4
uint8 CAR=5
uint8 TRUCK=6
