# Message describing the body of a person
uint32 GENDER_MALE = 1
uint32 GENDER_FEMALE = 0
uint32 NO_GENDER = 2

uint32 NO_AGE = 0

# Header
Header header

# The id of a person
string id

# Pose of the person with respect to the frame base_link
# as explained at http://www.ros.org/reps/rep-0105.html#base-link
geometry_msgs/Pose2D pose

# The probability with which this is a person
float32 person_confidence

# Height of the person in meters.
float32 height

# Width of a person: from shoulder to shoulder (not thickness)
float32 width

# Gender of the person; 1:male, 0:female
uint32[] gender

# Age of the person
uint32[] age

# The face of a person
nao_interaction_msgs/Face face

# Emotion associated with a person
nao_interaction_msgs/Emotion emotion
