import matplotlib.pyplot as plt
from Bezier import Bezier
from Bezier import Point

fig, axis = plt.subplots(2, 2)

RESOLUTION = 0.001

control_values = [Point(0, 0), Point(2,0.25), Point(0,1.5),Point(0,0.25)]


b = Bezier(control_values, RESOLUTION)
b.calculate_bezier_derivative()

b.calculate_offset(0.1)
b.calculate_distances()


axis[0][0].set_title("Bezier Curve with offsets")
axis[0][0].scatter(b.get_x(b.point_storage), b.get_y(b.point_storage), alpha=0.15)

# Note that the left/right flips when the curve turns 180 degrees! This means if the colors were to be different it wouldn't
# look right. This will need to be fixed eventually.
axis[0][0].scatter(b.get_x(b.left_offset_storage), b.get_y(b.left_offset_storage), alpha=0.15, color="RED")
axis[0][0].scatter(b.get_x(b.right_offset_storage), b.get_y(b.right_offset_storage), alpha=0.15, color="RED")

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
