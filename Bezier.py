import math


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def add(self, other):
        return Point(self.x + other, self.y + other)

    def distance_to(self, other):
        return math.hypot(abs(other.x-self.x), abs(other.y-self.y))

    def __add__(self, other):
        return Point(self.x + other.x, self.y + other.y)

    def mul(self, other):
        return Point(self.x * other, self.y * other)

    def __str__(self):
        return "X: " + str(self.x) + " Y: " + str(self.y) + "\n"


class Bezier:
    def __init__(self, control_values, resolution=0.001):
        # Storage of bezier curve points
        self.point_storage = []

        # Store the derivative of the entire line
        self.velocity = []
        self.total_derivative = []
        self.angles = []

        # Store the derivative of the x and y components, distance from (0,0) is total derivative
        self.derivative_component_storage = []

        # The offsets from the center line on the left and right and their angles and velocities
        self.left_offset_storage = []
        self.right_offset_storage = []

        self.left_offset_velocity = []
        self.left_angles = []

        self.right_offset_velocity = []
        self.right_angles = []

        # List of distances for each increase in resolution time
        self.distances = []

        # Info for equally spaced bezier curve
        self.equally_spaced_times = []
        self.equally_spaced_points = []
        self.equally_spaced_velocity = []

        # Provided control values
        self.control_values = control_values

        # The resolution of the bezier curve
        self.resolution = resolution

        # A list of times one less than 1/resolution long
        self.time_list = []
        for current_time in range(0, int(1 / self.resolution) - 1, 1):
            self.time_list.append(current_time * self.resolution)

        # Create the bezier curve
        for current_time in range(0, int(1 / self.resolution), 1):
            current_point = Bezier.recursive_lerp(self.control_values, current_time * self.resolution)
            self.point_storage.append(current_point)

    @staticmethod
    def lerp(point0, point1, t):
        """
        Perform linear interpolation between two points. ex. t=0.5 is halfway between them
        :param point0:
        :param point1:
        :param t: the time to interpolate at
        :return: the linear interpolated point
        """
        return point0.mul(1 - t) + point1.mul(t)

    @staticmethod
    def recursive_lerp(points, time):
        """
        Recursively perform linear interpolation between a list of control points given a time along the bezier curve (t is 0-1)
        :param points: The list of control points to perform recursive linear interpolation on
        :param time: the moment in time to grab the current point
        :return: The point into the curve based on the time and set of control points
        """
        if len(points) == 2:
            return Bezier.lerp(points[0], points[1], time)

        index = 0
        list_of_lerps = []
        while index < len(points) - 1:
            point1 = Bezier.lerp(points[index], points[index + 1], time)
            list_of_lerps.append(point1)
            index += 1

        return Bezier.recursive_lerp(list_of_lerps, time)

    @staticmethod
    def get_x(points):
        x_list = []
        for point in points:
            x_list.append(point.x)
        return x_list

    @staticmethod
    def get_y(points):
        y_list = []
        for point in points:
            y_list.append(point.y)
        return y_list

    # Returns index of x in arr if present, else -1
    @staticmethod
    def binary_search(arr, low, high, x):
        # Check base case
        if high >= low:

            mid = (high + low) // 2

            # If element is present at the middle itself
            if arr[mid - 1] < x < arr[mid + 1]:
                return mid

            # If element is smaller than mid, then it can only
            # be present in left subarray
            elif arr[mid] > x:
                return Bezier.binary_search(arr, low, mid - 1, x)

            # Else the element can only be present in right subarray
            else:
                return Bezier.binary_search(arr, mid + 1, high, x)

        else:
            # Element is not present in the array
            return -1

    @staticmethod
    def divide(n, d):
        return n / d if d else 0

    # Calculate the derivative of the bezier in x and y, the velocity, and the total (or raw) derivative of the curve
    def calculate_bezier_derivative(self):
        for i in range(0, len(self.point_storage) - 1):
            # Calculate the x and y velocity components (first deriv)
            derivative_x = (self.point_storage[i + 1].x - self.point_storage[i].x) / self.resolution
            derivative_y = (self.point_storage[i + 1].y - self.point_storage[i].y) / self.resolution
            self.derivative_component_storage.append(Point(derivative_x, derivative_y))

            # Calculate the total derivative and angle of the bezier curve (magnitude of (derivative_x, derivative_y)
            velocity = math.sqrt(derivative_x ** 2 + derivative_y ** 2)
            self.velocity.append(velocity)

            # Calculate the total derivative
            self.total_derivative.append(self.divide((derivative_y * self.resolution), (derivative_x * self.resolution)))

            # Calculate the current angle
            self.angles.append(math.degrees(math.atan2((derivative_y * self.resolution),
                                                       (derivative_x * self.resolution))))

    # Create the distance table
    def calculate_distances(self):
        cumulative_distance = 0
        for i in range(0, len(self.point_storage) - 1):
            distance = math.sqrt((self.point_storage[i + 1].x - self.point_storage[i].x) ** 2 + (
                    self.point_storage[i + 1].y - self.point_storage[i].y) ** 2)
            cumulative_distance += distance
            self.distances.append(cumulative_distance)

    def find_time_for_distance(self, distance):
        index = self.binary_search(self.distances, 0, len(self.distances) - 1, distance)
        time = index * self.resolution
        return time

    def find_point_for_distance(self, distance):
        index = self.binary_search(self.distances, 0, len(self.distances) - 1, distance)
        return self.point_storage[index]

    def compose_equal_distance_bezier(self):
        distance = 0
        while distance < self.distances[-1]:
            self.equally_spaced_times.append(self.find_time_for_distance(distance))
            self.equally_spaced_points.append(self.find_point_for_distance(distance))
            distance += 0.05

    def calculate_equal_distance_velocity(self):
        for i in range(0, len(self.equally_spaced_points) - 1):
            # Calculate the x and y velocity components (first deriv)
            derivative_x = (self.equally_spaced_points[i + 1].x - self.equally_spaced_points[i].x) / (
                        self.equally_spaced_times[i + 1] - self.equally_spaced_times[i])
            derivative_y = (self.equally_spaced_points[i + 1].y - self.equally_spaced_points[i].y) / (
                        self.equally_spaced_times[i + 1] - self.equally_spaced_times[i])

            # Calculate the total derivative and angle of the bezier curve (magnitude of (derivative_x, derivative_y)
            velocity = math.sqrt(derivative_x ** 2 + derivative_y ** 2)
            self.equally_spaced_velocity.append(velocity)

    # Store the points for each offset into storage - TODO: Fix the flipping at 0 degrees
    def calculate_offset(self, offset):
        sign = 1
        for i in range(0, len(self.total_derivative)):
            normal = -1 / (self.total_derivative[i])

            x0 = self.point_storage[i].x
            y0 = self.point_storage[i].y

            angle = math.atan(normal)

            x1 = offset * math.cos(angle) + x0
            y1 = offset * math.sin(angle) + y0

            x2 = offset * -math.cos(angle) + x0
            y2 = offset * -math.sin(angle) + y0

            self.left_offset_storage.append(Point(x1, y1))
            self.right_offset_storage.append(Point(x2, y2))

    # Calculate the velocity and angle over time of the offsets
    def calculate_offset_velocity_angles(self):
        b_left = Bezier(self.control_values, self.resolution)
        b_right = Bezier(self.control_values, self.resolution)

        b_left.point_storage = self.left_offset_storage
        b_right.point_storage = self.right_offset_storage

        b_left.calculate_bezier_derivative()
        b_right.calculate_bezier_derivative()

        return b_left.velocity  # , b_left.angles, b_right.velocity, b_right.angles
