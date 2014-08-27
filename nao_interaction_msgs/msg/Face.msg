# Message describing the face of a person.
float32 NO_HEAD = 0.0
float32 NO_GAZE = 0.0

# Header
Header header

# The id of a person
string id

# Pose of the face
geometry_msgs/Pose pose

# The height at which the bottom of the face is, in meters
float32 height

# The image of the face
sensor_msgs/Image image

# The direction of the head
geometry_msgs/Vector3 head

# The direction of the gaze
geometry_msgs/Vector3 gaze

# 12 characteristic points of the face
geometry_msgs/Point[] face_points
