# pick, place
string mode

# 夹取时的姿态和位置
geometry_msgs/Point position

float64 pitch
float64 align_angle

# 接近时的距离和方向
geometry_msgs/Vector3 grasp_approach

# 撤离时的距离和方向
geometry_msgs/Vector3 grasp_retreat

# 夹取时夹持器姿态
uint16 grasp_posture

# 夹取前夹持器姿态
uint16 pre_grasp_posture
