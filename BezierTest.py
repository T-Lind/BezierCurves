import math

import matplotlib.pyplot as plt
import numpy as np

from Bezier import Bezier
from Bezier import Point

fig, axis = plt.subplots(2, 2)

RESOLUTION = 0.001

control_values = [Point(0, 0), Point(1.5, 0.25), Point(0, 1.5), Point(0, 0.25)]

b = Bezier(control_values, RESOLUTION)
b.calculate_bezier_derivative()

b.calculate_offset(0.1)
b.calculate_distances()

axis[0][0].set_title("Bezier Curve with offsets")
axis[0][0].scatter(b.get_x(b.point_storage), b.get_y(b.point_storage), alpha=0.15)

axis[0][0].scatter([pt.x for pt in control_values], [pt.y for pt in control_values], color="green")

# Plot tangent lines
for ctrl_point in control_values:
    closest_idx = 0
    closest_distance = 10000

    # Get the closest point
    for i in range(len(b.point_storage)):
        point = b.point_storage[i]
        distance = point.distance_to(ctrl_point)
        if distance < closest_distance:
            closest_distance = distance
            closest_idx = i-1

    closest_pt = b.point_storage[closest_idx]

    # find the angle in radians at this point
    angle = math.radians(b.angles[closest_idx])
    slope = math.tan(angle)
    intercept = ctrl_point.y - slope * ctrl_point.x

    def line(x, x1, y1):
        return slope * (x - x1) + y1


    xrange = np.linspace(ctrl_point.x-10, ctrl_point.x+10, 10)
    axis[0][0].plot(xrange, line(xrange, ctrl_point.x, ctrl_point.y), "C1--")
# Note that the left/right flips when the curve turns 180 degrees! This means if the colors were to be different it wouldn't
# look right. This will need to be fixed eventually.
axis[0][0].scatter(b.get_x(b.left_offset_storage), b.get_y(b.left_offset_storage), alpha=0.15, color="RED")
axis[0][0].scatter(b.get_x(b.right_offset_storage), b.get_y(b.right_offset_storage), alpha=0.15, color="RED")

axis[0][0].set_xlim(-0.25, 2)
axis[0][0].set_ylim(-0.25, 2)

axis[0][1].set_title("X and Y first derivative output")
axis[0][1].plot(b.get_x(b.derivative_component_storage), b.get_y(b.derivative_component_storage))

axis[1][0].set_title("Angle in degrees of the bezier curve")
axis[1][0].scatter(b.time_list, b.angles, alpha=0.15)

# axis[1][0].scatter(b.time_list, left_v, color="RED")
# axis[1][0].scatter(b.time_list, right_v, color="GREEN")

axis[1][1].set_title("Non-normalized velocity")
axis[1][1].plot(b.time_list, b.velocity, alpha=0.15)

#
# axis[1][0].set_title("Distance over time")
# axis[1][0].plot(b.time_list, b.distances)
#
# axis[2][1].set_title("Equal distance bezier")
# axis[2][1].plot(b.get_x(b.equally_spaced_points), b.get_y(b.equally_spaced_points))
#
# axis[3][1].set_title("Equal distance bezier velocity")
# axis[3][1].scatter(b.equally_spaced_times[0:-2], b.equally_spaced_velocity[1:])


plt.show()
