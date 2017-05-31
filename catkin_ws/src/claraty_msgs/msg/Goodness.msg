# The 2D goodness map

std_msgs/Header header 

# MetaData for the map
nav_msgs/MapMetaData info

# The map data
float32[] goodness   # Goodness in the range [0, 1]
float32[] certainty  # Certainty in the range [0, 1]
float32[] height     # Elevation from the reference frame
