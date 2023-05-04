import math
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from functools import partial
import matplotlib.patches as patches


# vehicle measurements in cm

AXL_DISTANCE = 3.5 # distance from wheel to center
SENSOR_LENGTH = 5 # perpendicular distance from sensor to center line
SENSOR_WIDTH = 2.5 # distance between both sensors

ROBOT_WIDTH = 7
ROBOT_HEIGHT = 11


# wheel velocity in cm/s
MAX_WHEEL_VELOCITY = 29


# ellipse measurements in cm

ELLIPSE_THICKNESS = 1.5
ELLIPSE_VISUAL_THICKNESS = 3
X_CENTER = 0
Y_CENTER = 7.5
ELLIPSE_A = 12.5
ELLIPSE_B = 7.5


def vehicle_model(uk, xk, timestep):

    vk = uk[0]
    turn_rate = uk[1]

    x = xk[0]
    y = xk[1]
    heading = xk[2]

    xk[0] = x + timestep * vk * math.cos(heading)
    xk[1] = y + timestep * vk * math.sin(heading)

    xk[2] = heading + timestep * turn_rate

    return xk


def SensorModel(xk):

    # print(xk[0], xk[1])

    sl = [0, 0]
    sr = [0, 0]

    # length and width of Thing :


    r = math.sqrt(SENSOR_LENGTH**2 + (SENSOR_WIDTH/2)**2)
    theta = math.atan((SENSOR_WIDTH/2)/SENSOR_LENGTH)

    # sl[0] = xk[0] + r * math.cos((xk[2] * math.pi/180 + theta))
    # sl[1] = xk[1] + r * math.sin((xk[2] * math.pi/180 + theta))

    # sr[0] = xk[0] + r * math.cos((xk[2] * math.pi/180 - theta))
    # sr[1] = xk[1] + r * math.sin((xk[2] * math.pi/180 - theta))


    sl[0] = xk[0] + r * math.cos((xk[2] + theta))
    sl[1] = xk[1] + r * math.sin((xk[2] + theta))

    sr[0] = xk[0] + r * math.cos((xk[2] - theta))
    sr[1] = xk[1] + r * math.sin((xk[2] - theta))




    return sl, sr



def TrackModel(sl, sr, a, b, x0, y0):


    # distance from center of ellipse to point
    # dl = math.sqrt((sl[0] - x0)**2 + (sl[1] - y0)**2)

    # find the equation of line from center to given point:

    # gradient:

    try:

        ml = (y0 - sl[1]) / (x0 - sl[0])


        # now find b from y = mx + b

        constl = y0 - ml * x0

        # we now have the equation of the line from the center of the ellipse to the given point


        # now find the intersection of the ellipse and this line:

        # solve the equations simultaneously to do this

        # y = mx + const
        # -mx + y = const

        # (x - x0)**2/a**2 + (y - y0)**2/b**2 - 1

        # get it into quadratic form after subbing line equation into ellipse equation:

        # ax^2 + bx + c = 0


        # where a = (b**2 + a**2 * m**2), b = (-2 * b**2 * x0 + a**2 * 2 * (c - y0) * m * x), c = b**2 * x0**2 + a**2 * (c - y0)**2 - a**2 * b**2


        # x**2 * (b**2 + a**2 * m**2) + x * (-2 * b**2 * x0 + a**2 * 2 * (c - y0) * m * x) + b**2 * x0**2 + a**2 * (c - y0)**2 - a**2 * b**2 = 0


        al = (b**2 + a**2 * ml**2)
        bl = (-2 * b**2 * x0 + a**2 * 2 * (constl - y0) * ml)
        cl = b**2 * x0**2 + a**2 * (constl - y0)**2 - a**2 * b**2

        # now we can solve quadratically to find the intersect/s:


        xlp = (-bl + math.sqrt(bl**2 - 4 * al * cl)) / (2 * al)
        xln = (-bl - math.sqrt(bl**2 - 4 * al * cl)) / (2 * al)
        
        ylp = ml * xlp + constl
        yln = ml * xln + constl




    except(ZeroDivisionError):

        # the line between the 2 points is vertical

        ylp = y0 + b
        yln = y0 - b

        xlp = x0
        xln = x0



    try:

        mr = (y0 - sr[1]) / (x0 - sr[0])
        constr = y0 - mr * x0

        ar = (b**2 + a**2 * mr**2)
        br = (-2 * b**2 * x0 + a**2 * 2 * (constr - y0) * mr)
        cr = b**2 * x0**2 + a**2 * (constr - y0)**2 - a**2 * b**2

        xrp = (-br + math.sqrt(br**2 - 4 * ar * cr)) / (2 * ar)
        xrn = (-br - math.sqrt(br**2 - 4 * ar * cr)) / (2 * ar)
        
        yrp = mr * xrp + constr
        yrn = mr * xrn + constr

    except(ZeroDivisionError):

        yrp = y0 + b
        yrn = y0 - b

        xrp = x0
        xrn = x0



    # now we find the distance from the given point to each of these points:

    dlp = math.sqrt((sl[0] - xlp)**2 + (sl[1] - ylp)**2)
    dln = math.sqrt((sl[0] - xln)**2 + (sl[1] - yln)**2)

    drp = math.sqrt((sr[0] - xrp)**2 + (sr[1] - yrp)**2)
    drn = math.sqrt((sr[0] - xrn)**2 + (sr[1] - yrn)**2)


    where_l = where_point(sl[0], sl[1], a, b, x0, y0)
    where_r = where_point(sr[0], sr[1], a, b, x0, y0)

    # print(where_l, where_r)


    rl = min(dlp, dln)
    rr = min(drp, drn)

    # if rl < ELLIPSE_THICKNESS / 2:
    #     rl = 0
    # else:
    #     rl = rl - ELLIPSE_THICKNESS / 2

    # if rr < ELLIPSE_THICKNESS / 2:
    #     rr = 0
    # else:
    #     rr = rr - ELLIPSE_THICKNESS / 2



    if where_l < 0:
        rl = -rl

    if where_r < 0:
        rr = -rr



    return rl, rr



def ControlModel(sl, sr):


    vl = MAX_WHEEL_VELOCITY
    vr = MAX_WHEEL_VELOCITY

    if sr == 0:
        vr = 0 
    
    if sl == 0:
        vl = 0
    
    return vl, vr



def ControlModelProportional(dl, dr):


    # we have to make up a formula to convert distance into wheel speeds


    # outside wheel should always be faster

    vl = MAX_WHEEL_VELOCITY
    vr = MAX_WHEEL_VELOCITY



    if dl < 0 and abs(dl) < 2: # left wheel is inside the ellipse 
        vl = MAX_WHEEL_VELOCITY * 2/math.pi * math.atan(abs(dl) * 3)

    if dr < 0 and abs(dl) < 2:
        vr = MAX_WHEEL_VELOCITY * 2/math.pi * math.atan(abs(dr) * 3)


    return vl, vr






def where_point(x, y, a, b, x0, y0):

    return (x - x0)**2/a**2 + (y - y0)**2/b**2 - 1


def where_point_thick(x, y, a, b, x0, y0, thickness):

    # now we have 2 elipses where a and b of each are offset by the given thickness

    a_inner = a - thickness / 2
    b_inner = b - thickness / 2

    a_outer = a + thickness / 2
    b_outer = b + thickness / 2

    result_inner = (x - x0)**2/a_inner**2 + (y - y0)**2/b_inner**2 - 1
    result_outer = (x - x0)**2/a_outer**2 + (y - y0)**2/b_outer**2 - 1

    if result_outer > 0:
        return result_outer
    
    if result_inner < 0:
        return result_inner
    
    return 0



def simulate(d, vr, vl, x, y, heading, time, time_step):

    t = 0

    turn_rate = (vr - vl) / d
    vk = (vr + vl) / 2

    uk = [vk, turn_rate]
    xk = [x, y, heading]

    while t < time:

        t += time_step

        xk = vehicle_model(uk, xk, time_step)

        x_poses.append(xk[0])
        y_poses.append(xk[1])

        # print_thing(x, y, pitchfork_k, t)

    ax.plot(x_poses, y_poses)



def elipse_simulate(x0, y0, a, b, thickness, step):

    x_range = [x0 - a - 5, x0 + a + 5]
    y_range = [y0 - b - 5, y0 + b + 5]

    x = x_range[0]
    y = y_range[0]

    while x < x_range[1]:

        while y < y_range[1]:

            x_poses.append(x)
            y_poses.append(y)

            result.append(where_point_thick(x, y, a, b, x0, y0, thickness))

            y += step


        x += step
        y = y_range[0]

    

# arrays to store the x and y positions of the vehicle

x_poses = []
y_poses = []


# array to store score of where point is // used in ellipse simulate
result = []

# elipse_simulate(0, 7.5, 12.5, 7.5, 1.5, 1)


# for i in range(len(x_poses)):

#     if result[i] == 0:
#         colour = 'green'
#     elif result[i] > 0:
#         colour = 'red'
#     else:
#         colour = 'blue'


#     plt.scatter(x_poses[i], y_poses[i], color = colour, s = 18)


# ELLIPSE_THICKNESS = 0.015
# X_CENTER = 0
# Y_CENTER = 0.075
# ELLIPSE_A = 0.125
# ELLIPSE_B = 0.075


xk = [X_CENTER, Y_CENTER + ELLIPSE_B, 0]



# simulate(AXL_DISTANCE, MAX_WHEEL_VELOCITY, 0, 0, 0, 0, 10, 0.1) # simulating part d


# simulate_full_bangbang([0, 15, 0], ELLIPSE_A, ELLIPSE_B, X_CENTER, Y_CENTER, ELLIPSE_THICKNESS, 10, 0.1)


# plotting the ellipse


def make_plot(xk, x_poses, y_poses, sl, sr):


    ax.cla()
    ax.grid(color='lightgray')
    ax.set_axisbelow(True)


    # creating the inner and outer ellipses:

    t = np.linspace(0, 2*math.pi, 100)

    x_inner = X_CENTER + (ELLIPSE_A - ELLIPSE_THICKNESS / 2)*np.cos(t)
    y_inner = Y_CENTER + (ELLIPSE_B - ELLIPSE_THICKNESS / 2)*np.sin(t)

    x_outer = X_CENTER + (ELLIPSE_A + ELLIPSE_THICKNESS / 2)*np.cos(t)
    y_outer = Y_CENTER + (ELLIPSE_B + ELLIPSE_THICKNESS / 2)*np.sin(t)


    # creating the track by filling space between both ellipses

    ax.fill_between(x_inner, y_outer, y_inner, facecolor="black")

    ax.fill_betweenx(y_outer, x_outer, x_inner, facecolor="black")



    # plotting path of vehicle
    ax.plot(x_poses, y_poses, color="blue", zorder=1)

    # creating outline of vehicle
    rect = patches.Rectangle((xk[0] - ROBOT_HEIGHT/2, xk[1] - ROBOT_WIDTH/2), ROBOT_HEIGHT, ROBOT_WIDTH, linewidth=1, edgecolor="blue", facecolor="none", angle=xk[2] * 180/math.pi, rotation_point='center')

    ax.add_patch(rect)

    # plotting points of vehicle position, and sensor positions

    ax.scatter(xk[0], xk[1], color="blue", s=12)
    ax.scatter(sl[0], sl[1], color="green", s=15, zorder=2)
    ax.scatter(sr[0], sr[1], color="red", s=15, zorder=2)


    # putting a point in each corner +5 to stop annoying resizing

    #bottom left
    ax.scatter(X_CENTER - ELLIPSE_A - 5, Y_CENTER - ELLIPSE_B - 5, color="none")

    # top right

    ax.scatter(X_CENTER + ELLIPSE_A + 5, Y_CENTER + ELLIPSE_B + 5, color="none")






def animate_bangbang(frame, xk, timestep):


    print("Heading:", xk[2])

    sl, sr = SensorModel(xk)

    # dl, dr = TrackModel(sl, sr, a, b, x0, y0)

    # vl, vr = ControlModelProportional(dl, dr)
    make_plot(xk, x_poses, y_poses, sl, sr)


    vl, vr = ControlModel(where_point_thick(sl[0], sl[1], ELLIPSE_A, ELLIPSE_B, X_CENTER, Y_CENTER, ELLIPSE_THICKNESS), 
                          where_point_thick(sr[0], sr[1], ELLIPSE_A, ELLIPSE_B, X_CENTER, Y_CENTER, ELLIPSE_THICKNESS))


    turn_rate = (vr - vl) / AXL_DISTANCE
    vk = (vr + vl) / 2

    uk = [vk, turn_rate]


    xk = vehicle_model(uk, xk, timestep)

    x_poses.append(xk[0])
    y_poses.append(xk[1])






def animate_proportional(frame, xk, timestep):

    sl, sr = SensorModel(xk)

    make_plot(xk, x_poses, y_poses, sl, sr)

    dl, dr = TrackModel(sl, sr, ELLIPSE_A, ELLIPSE_B, X_CENTER, Y_CENTER)

    vl, vr = ControlModelProportional(dl, dr)

    turn_rate = (vr - vl) / AXL_DISTANCE
    vk = (vr + vl) / 2

    uk = [vk, turn_rate]


    xk = vehicle_model(uk, xk, timestep)

    x_poses.append(xk[0])
    y_poses.append(xk[1])



fig, ax = plt.subplots()


ani = FuncAnimation(plt.gcf(), partial(animate_proportional, xk=xk, timestep=0.01), interval=1, cache_frame_data=False)


# ani = FuncAnimation(plt.gcf(), partial(animate_bangbang, xk=xk, timestep=0.01), interval=10, cache_frame_data=False)



# simulate(AXL_DISTANCE, MAX_WHEEL_VELOCITY, 0, 0, 0, 0, 10, 0.01)



ax.set_aspect('equal')
plt.show()




# ani = FuncAnimation(plt.gcf(), simulate_full_proportional())



# #plotting:



# x = 12
# y = 1000

# print(TrackModel([x, y], [x, y], 12.5, 7.5, 0, 7.5))
# print(where_point(x, y, 12.5, 7.5, 0, 7.5))


# plt.scatter(x, y, color = 'black', s = 25)



# for full_simulate_proportional

# for i in range(len(x_poses)):
#     plt.scatter(x_poses[i], y_poses[i])






