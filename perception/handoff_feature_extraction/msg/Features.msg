# Features for a single cluster

Header header

# Object transform, however calculated
geometry_msgs/Transform transform


#################################
# Raw point Related

# Cluster centroid, min, max and number of points
geometry_msgs/Vector3 points_centroid
geometry_msgs/Vector3 points_min   #<pc min x, pc min y, pc min z>
geometry_msgs/Vector3 points_max   #<pc max x, pc max y, pc max z>
float64 num_points

# Average color (RGBA nad hue)
std_msgs/ColorRGBA rgba_color
float64 hue

#################################
#Bounding box

#position wrt sensor and angle wrt table normal
geometry_msgs/Vector3 bb_center
float64 bb_angle

# Bounding box dimensions
geometry_msgs/Vector3 bb_dims

#################################
# Hue-Saturation histogram

uint32 hs_features_size
float64[] hs_feature_vector

#################################
# VFH features

uint32 vfh_features_size
float64[] vfh_feature_vector

#################################
# Shape histogram using CVFH

uint32 cvfh_features_size
float64[] cvfh_feature_vector

#################################
# Shape histogram using FPFH

uint32 fpfh_features_size
float64[] fpfh_feature_vector

#################################

#Other features, etc. Unpacking will be on the other side

uint32 other_features_size
float64[] data
