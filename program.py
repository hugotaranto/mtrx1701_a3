import math
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from functools import partial
import matplotlib.patches as patches

# vehicle measurements in cm

AXL_DISTANCE = 3.5 # distance from wheel to center
SENSOR_LENGTH = 4 # perpendicular distance from sensor to center line
SENSOR_WIDTH = 2.5 # distance between both sensors

ROBOT_WIDTH = 7
ROBOT_HEIGHT = 10


# wheel velocity in cm/s
MAX_WHEEL_VELOCITY = 29


# ellipse measurements in cm

ELLIPSE_THICKNESS = 1.5
ELLIPSE_VISUAL_THICKNESS = 3
X_CENTER = 0
Y_CENTER = 7.5
ELLIPSE_A = 12.5
ELLIPSE_B = 7.5

# timestep used throughout:

TIME_STEP = 0.01 # 100 steps per second

def VehicleModel(uk, xk):

    vk = uk[0]
    turn_rate = uk[1]

    x = xk[0]
    y = xk[1]
    heading = xk[2]

    xk[0] = x + TIME_STEP * vk * math.cos(heading)
    xk[1] = y + TIME_STEP * vk * math.sin(heading)

    xk[2] = heading + TIME_STEP * turn_rate

    return xk


def SensorModel(xk):

    # arrays storing x and y coordinates respectively:
    sl = [0, 0] 
    sr = [0, 0]

    # setting up r and theta:
    r = math.sqrt(SENSOR_LENGTH**2 + (SENSOR_WIDTH/2)**2)
    theta = math.atan((SENSOR_WIDTH/2)/SENSOR_LENGTH)

    # calculating coordinates for both sensors:
    sl[0] = xk[0] + r * math.cos((xk[2] + theta))
    sl[1] = xk[1] + r * math.sin((xk[2] + theta))

    sr[0] = xk[0] + r * math.cos((xk[2] - theta))
    sr[1] = xk[1] + r * math.sin((xk[2] - theta))

    return sl, sr


def TrackModel(sl, sr, a, b, x0, y0, thickness):

    dl = where_point_thick(sl[0], sl[1], a, b, x0, y0, thickness)
    dr = where_point_thick(sr[0], sr[1], a, b, x0, y0, thickness)

    return dl, dr


def ControlModel(dl, dr):

    vl = MAX_WHEEL_VELOCITY
    vr = MAX_WHEEL_VELOCITY

    if dr == 0:
        vr = 0 
    
    if dl == 0:
        vl = 0
    
    return vl, vr


def ControlModelProportional(dl, dr):

    # outside wheel should always be faster

    vl = MAX_WHEEL_VELOCITY
    vr = MAX_WHEEL_VELOCITY

    if dl < 0: # left wheel is inside the ellipse 
        vl = MAX_WHEEL_VELOCITY * 2/math.pi * math.atan(abs(dl) * 100) # gives a value from 0 to max wheel velocity

    if dr < 0: # right wheel is inside the ellipse
        vr = MAX_WHEEL_VELOCITY * 2/math.pi * math.atan(abs(dr) * 100)


    return vl, vr


def where_point(x, y, a, b, x0, y0):

    return (x - x0)**2/a**2 + (y - y0)**2/b**2 - 1


def where_point_thick(x, y, a, b, x0, y0, thickness):

    # now we have 2 elipses where a and b of each are offset by the given thickness / 2

    a_inner = a - thickness / 2
    b_inner = b - thickness / 2

    a_outer = a + thickness / 2
    b_outer = b + thickness / 2

    # now find the results of the function given the point and these 2 ellipses

    result_inner = (x - x0)**2/a_inner**2 + (y - y0)**2/b_inner**2 - 1
    result_outer = (x - x0)**2/a_outer**2 + (y - y0)**2/b_outer**2 - 1

    # if the result on the outer ellipse is > 0 
    # we know the point is outside the thick ellipse
    if result_outer > 0:
        return result_outer
    
    # if the result on the inner ellipse is < 0 
    # we know the point is inside the thick ellipse
    if result_inner < 0:
        return result_inner
    
    # otherwise the point is on the ellipse

    return 0


def simulate(vr, vl, x, y, heading, duration, ax):

    # calculate velocity and turn rate given wheel speeds:

    vk = (vr + vl) / 2
    turn_rate = (vr - vl) / AXL_DISTANCE
    
    # put results into arrays

    uk = [vk, turn_rate]
    xk = [x, y, heading]

    x_poses = []
    y_poses = []

    t = 0

    # loop through until t < given time, t is incremented by TIME_STEP every iteration

    while t < duration:

        t += TIME_STEP

        # put vehicle x and y coordinates into lists
        x_poses.append(xk[0])
        y_poses.append(xk[1])


        # update xk to be xk + 1
        xk = VehicleModel(uk, xk)

        
    # after loop has finished plot the stored x and y coordinates
    ax.plot(x_poses, y_poses)


def ellipse_simulate(x0, y0, a, b, gap, ax):

    # setting up the range of the x and y values, we take a and b and extend them by 5:

    x_range = [x0 - a - 5, x0 + a + 5]
    y_range = [y0 - b - 5, y0 + b + 5]

    x = x_range[0]
    y = y_range[0]

    # setting up arrays to store values

    x_poses = []
    y_poses = []
    result = []

    # now at each x value loop through all of the y values incrementing by the given gap

    while x < x_range[1]:

        while y < y_range[1]:

            # add the coordinates to the arrays

            x_poses.append(x)
            y_poses.append(y)

            # find the result of where the point is:

            result.append(where_point(x, y, a, b, x0, y0))

            y += gap

        # go to next x value and reset y to initial
        x += gap
        y = y_range[0]


    # now plot the values, if the result was on the ellipse plot a black point

    # if the point is outside the ellipse plot a reddish point

    # if the point is inside the ellipse plot a blueish point

    for i in range(len(x_poses)):

        if result[i] == 0: # on ellipse
            colour = 'black'
        elif result[i] > 0: # outside

            # we use this arctan function to give a result between 0 and 1

            colour = [(2/math.pi * math.atan(result[i] * 2), 0.5, 0)] 
        else: # inside
            colour = [(0, 0.5, 2/math.pi * math.atan(abs(result[i]) * 2))]

        ax.scatter(x_poses[i], y_poses[i], color=colour, s=18)


def ellipse_simulate_thick(x0, y0, a, b, gap, thickness, ax):

    # setting up the range of the x and y values, we take a and b and extend them by 5:

    x_range = [x0 - a - 5, x0 + a + 5]
    y_range = [y0 - b - 5, y0 + b + 5]

    x = x_range[0]
    y = y_range[0]

    # setting up arrays to store values

    x_poses = []
    y_poses = []
    result = []

    # now at each x value loop through all of the y values incrementing by the given gap

    while x < x_range[1]:

        while y < y_range[1]:

            # add the coordinates to the arrays

            x_poses.append(x)
            y_poses.append(y)

            # find the result of where the point is:

            result.append(where_point_thick(x, y, a, b, x0, y0, thickness))

            y += gap

        # go to next x value and reset y to initial
        x += gap
        y = y_range[0]


    # now plot the values, if the result was on the ellipse plot a black point

    # if the point is outside the ellipse plot a reddish point

    # if the point is inside the ellipse plot a blueish point

    for i in range(len(x_poses)):

        if result[i] == 0: # on ellipse
            colour = 'black'
        elif result[i] > 0: # outside

            # we use this arctan function to give a result between 0 and 1

            colour = [(2/math.pi * math.atan(result[i] * 2), 0.5, 0)] 
        else: # inside
            colour = [(0, 0.5, 2/math.pi * math.atan(abs(result[i]) * 2))]

        ax.scatter(x_poses[i], y_poses[i], color=colour, s=18)


def make_plot(xk, x_poses, y_poses, sl, sr, ax):

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
    rect = patches.Rectangle((xk[0] - ROBOT_HEIGHT/2, xk[1] - ROBOT_WIDTH/2), 
                             ROBOT_HEIGHT, ROBOT_WIDTH, linewidth=1, edgecolor="blue", 
                             facecolor="none", angle=xk[2] * 180/math.pi, 
                             rotation_point='center')

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



def animate_bangbang(frame, x_poses, y_poses, lap_frames, xk, ax):

    # this is keeping track of the number of frames that have ellapsed when
    # the robot passes through and x coordinate of 0
    if frame != 0:
        if xk[0] > 0 and x_poses[-1] < 0:
            lap_frames.append(frame)

    # append the current position onto the positions array
    x_poses.append(xk[0])
    y_poses.append(xk[1])

    # set the sensor positions
    sl, sr = SensorModel(xk)

    # now that we have sensor positions we can make the plot of the current frame.
    make_plot(xk, x_poses, y_poses, sl, sr, ax)

    # get the distance from each sensor to the ellipse:
    dl, dr = TrackModel(sl, sr, ELLIPSE_A, ELLIPSE_B, X_CENTER, Y_CENTER, 
                        ELLIPSE_THICKNESS)

    # calculate the velocities of the robot:
    vl, vr = ControlModel(dl, dr)

    # create uk given the velocities:
    turn_rate = (vr - vl) / AXL_DISTANCE
    vk = (vr + vl) / 2

    uk = [vk, turn_rate]

    # and calculate xk + 1
    xk = VehicleModel(uk, xk)

    # this will then be looped again with the new xk value for the next frame.


def animate_proportional(frame, x_poses, y_poses, lap_frames, xk, ax):

    # this is keeping track of the number of frames that have ellapsed when
    # the robot passes through and x coordinate of 0
    if frame != 0:
        if xk[0] > 0 and x_poses[-1] < 0:
            lap_frames.append(frame)

    # append the current position onto the positions array
    x_poses.append(xk[0])
    y_poses.append(xk[1])

    # set the sensor positions
    sl, sr = SensorModel(xk)

    # now that we have sensor positions we can make the plot of the current frame.
    make_plot(xk, x_poses, y_poses, sl, sr, ax)

    # get the distance from each sensor to the ellipse:
    dl, dr = TrackModel(sl, sr, ELLIPSE_A, ELLIPSE_B, X_CENTER, Y_CENTER, 
                        ELLIPSE_THICKNESS)

    # calculate the velocities of the robot:
    vl, vr = ControlModelProportional(dl, dr)

    # create uk given the velocities:
    turn_rate = (vr - vl) / AXL_DISTANCE
    vk = (vr + vl) / 2

    uk = [vk, turn_rate]

    # and calculate xk + 1
    xk = VehicleModel(uk, xk)

    # this will then be looped again with the new xk value for the next frame.

    
# initial xk value
xk = [0, 0, 0]
fig, ax = plt.subplots()

x_poses = []
y_poses = []

# array to store the frame numbers when the robot passes from a negative x value to a positive
lap_frames = []

# function from matplotlib.animation library to animate matplots

ani = FuncAnimation(plt.gcf(), partial(animate_proportional, 
                                       x_poses=x_poses, 
                                       y_poses=y_poses, 
                                       lap_frames=lap_frames, xk=xk, ax=ax), 
                                       interval=10, cache_frame_data=False)

ax.set_aspect("equal")
plt.show()


# this is printing out the times that each lap took.
for i in range(len(lap_frames)):
    print("time for loop {}: {}s".format(i + 1, lap_frames[i] * TIME_STEP))

