# 颜色标签，如'red1', 'green2', 'blue3'
# 垃圾标签, 如'BananaPeel'
string label

# 物体的物理世界中心点坐标
float64 x
float64 y

# 机械臂夹持器转到和物体平行的角度
int16 yaw

# 碰撞区域
hiwonder_interfaces/PixelPosition[] collision_roi

# 表示夹取的物体是否与周围的物体足够近，导致夹持器夹取时会产生碰撞
# 如果为真，则yaw不可用
bool collision
