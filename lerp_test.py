import matplotlib.pyplot as plt

RESOLUTION = 0.001


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return "X: "+str(self.x)+" Y: "+str(self.y)+"\n"

def lerp(point0, point1, t):
    x = (1 - t) * point0.x + t * point1.x
    y = (1 - t) * point0.y + t * point1.y
    return Point(x, y)


def recursive_lerp(points, time):
    if len(points) == 2:
        return lerp(points[0], points[1], time)

    index = 0
    list_of_lerps = []
    while index < len(points) - 1:
        point1 = lerp(points[index], points[index + 1], time)
        list_of_lerps.append(point1)
        index += 1

    return recursive_lerp(list_of_lerps, time)


control_values = [Point(0, 0), Point(3, 2), Point(5, 2.5)]

point = recursive_lerp(control_values, 0.1)
print(point)