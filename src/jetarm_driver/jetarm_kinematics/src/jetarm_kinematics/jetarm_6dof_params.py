'''
Modified DH
----------------------------------------------
i | α(i-1) | a(i-1) |       θ(i)      | d(i) |
----------------------------------------------
1 |   0°   |   0    |  θ1(-120, 120)  |   0  |
----------------------------------------------
2 |  -90°  |   0    |  θ2(-180, 0)    |   0  |
----------------------------------------------
3 |   0°   | link1  |  θ3(-120, 120)  |   0  |
----------------------------------------------
4 |   0°   | link2  |  θ4(-200, 20)   |   0  |
----------------------------------------------
5 |  -90°  |   0    |  θ5(-120, 120)  |   0  |
----------------------------------------------
'''

# 连杆长度(m)(link length(m))
# 底座的高度，这里把第一个坐标系和第二个坐标的原点重合到一起了(height of the base, the origins of the first and second coordinate systems are aligned together)
base_link = 0.10314916202

link1 = 0.12941763737
link2 = 0.12941763737

# 计算tool_link时取值为link3 + tool_link，因为把末端的坐标系原点和前一个重合到一起了(The calculation of tool_link is obtained by adding link3 and tool_link, because the origin of the end effector coordinate system is aligned with the previous one)
# 这里的tool_link指实际上的夹持器长度(The tool_link refers to the actual length of the gripper)
link3 = 0.05945583202
tool_link = 0.112

# 各关节角度限制，取决于是否碰撞以及舵机的转动范围(The joint angle limits depend on whether there is a collision and the rotation range of the servo)
# 多加0.2为了防止计算时数值的不稳定，会比设定值大一点点(Adding 0.2 is to prevent numerical instability during calculation, which may be slightly larger than the set value)
joint1 = [-120.2, 120.2]
joint2 = [-180.2, 0.2]
joint3 = [-120.2, 120.2]
joint4 = [-200.2, 20.2]
joint5 = [-120.2, 120.2]

# 舵机脉宽范围，中位值，对应的角度范围，中位值(The servo pulse width range, median value, corresponding angle range, and median value)
joint1_map = [0, 1000, 500, -120, 120, 0]
joint2_map = [0, 1000, 500, 30, -210, -90]
joint3_map = [0, 1000, 500, 120, -120, 0]
joint4_map = [0, 1000, 500, 30, -210, -90]
joint5_map = [0, 1000, 500, -120, 120, 0]
