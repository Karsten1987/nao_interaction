# Message describing the emotion of a person.

# Header
Header header

# Valence emotion of the person [-100;100]; Format : [value; confidence]
float32[] valence

# Attention of the person toward the robot [0;100]; Format : [value; confidence]
float32[] attention

# Synergology
string synergology

# Smile degree of the person [0;1]; Format : [value; confidence]
float32[] smile

# Face expressions of the person; Format : [neutral; joy; sad; anger]
float32[] expressions
