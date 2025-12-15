import numpy as np

while True:
    angle = float(input("Enter angle in degrees: "))
    angle_rad = np.rad2deg(angle)
    print(f"Angle in radians: {angle_rad}")