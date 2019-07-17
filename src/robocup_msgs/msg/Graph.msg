#std_msgs/Header         header
geometry_msgs/Pose[]    new_nodes
geometry_msgs/Pose[]    removed_nodes

# edges are encoded such that the connected edges are consecutive
# from-to, from-to, ...
geometry_msgs/Pose[]    new_edges_from
geometry_msgs/Pose[]    new_edges_to
geometry_msgs/Pose[]    removed_edges_from
geometry_msgs/Pose[]    removed_edges_to

geometry_msgs/Pose[]    path
