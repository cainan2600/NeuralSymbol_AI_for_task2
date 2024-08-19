import math

# def luoxuan(theta):
#     a = 0
#     b = 0.05 / (2 * math.pi)
#     theta_1 = theta * math.pi / 180
#     x = (a + (b * theta_1)) * (math.cos(theta_1))
#     y = (a + (b * theta_1)) * (math.sin(theta_1))
#     return x, y


# print(luoxuan(30), luoxuan(60), luoxuan(90), luoxuan(120), luoxuan(150), luoxuan(180),
#         luoxuan(210), luoxuan(240), luoxuan(270), luoxuan(300), luoxuan(330), luoxuan(360))


# 

def get_search_trajectory(attempt, radius, rotation=0):

    radius = radius

    a = 0
    b = 0.05 / (2 * math.pi)
    delta_angle = 30
    scale_angle = delta_angle * math.pi / 180

    print("get_search_trajectory")

    for i in range(1, 5):
        print("i={}".format(i))
        b = b * i
        for j in range(int(360 * attempt / delta_angle)):
            tgt_pose_in_world_frame = 7
            if tgt_pose_in_world_frame is None:
                print("max is and no trajectory when 2pi")
                i += 1
            else:
                print(tgt_pose_in_world_frame)
                # tgt_pose_in_world_frame = None
        break



print(get_search_trajectory(attempt=1, radius=1))