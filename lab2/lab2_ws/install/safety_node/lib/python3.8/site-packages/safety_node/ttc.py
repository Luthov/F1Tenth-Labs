import math

def calculate_ttc(ranges, longitudinal_vel):
    angle = 0
    angle_increment = math.pi/720
    r_dot_list = []
    iTTC = []
    for i in range(len(ranges)):
        r_dot_list.append(longitudinal_vel * math.cos(angle))
        angle += angle_increment
    r_dot = max(r_dot_list)
    
    for i in ranges:
        iTTC.append(i / r_dot)

    return max(iTTC)

print(calculate_ttc([10.0, 30.0, 60.0], 1.0))
