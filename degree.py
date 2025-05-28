from math import sin, cos, radians

def point_on_circle(degree, radius=400, center=(-650, -133)):
    theta = radians(degree)
    x = center[0] + radius * sin(theta)
    y = center[1] + radius * cos(theta)
    return (round(x, 2), round(y, 2))


for alpha in range (100, 70, -5):
    print(180 - alpha)
    print(point_on_circle(alpha))