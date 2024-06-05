import math

# Constants
f = 458.61  # Example value for f
e = 50   # Example value for e
rf = 200  # Example value for rf
re = 400  # Example value for re
pi = math.pi

# Helper function to calculate angle theta1 (for YZ-plane)
def delta_calcAngleYZ(x0, y0, z0):
    y1 = -0.5 * 0.57735 * f  # f/2 * tg 30
    y0 -= 0.5 * 0.57735 * e   # shift center to edge
    a = (x0*x0 + y0*y0 + z0*z0 + rf*rf - re*re - y1*y1)/(2*z0)
    b = (y1 - y0)/z0
    d = -(a + b*y1)**2 + rf * (b*b*rf + rf)
    if d < 0:
        return -1, 0  # Non-existing point
    yj = (y1 - a*b - math.sqrt(d))/(b*b + 1)  # Choosing outer point
    zj = a + b*yj
    theta = 180.0 * math.atan(-zj/(y1 - yj))/pi + (180.0 if yj > y1 else 0.0)
    return 0, theta

# Inverse kinematics: (x0, y0, z0) -> (theta1, theta2, theta3)
# Returned status: 0=OK, -1=non-existing position
def delta_calcInverse(x0, y0, z0):
    print(f"Calculando inversa para x0={x0}, y0={y0}, z0={z0}")
    theta1 = theta2 = theta3 = 0
    status, theta1 = delta_calcAngleYZ(x0, y0, z0)
    print(f"theta1: status={status}, angle={theta1}")
    if status == 0:
        status, theta2 = delta_calcAngleYZ(x0*math.cos(2*math.pi/3) + y0*math.sin(2*math.pi/3), y0*math.cos(2*math.pi/3) - x0*math.sin(2*math.pi/3), z0)
        print(f"theta2: status={status}, angle={theta2}")
    if status == 0:
        status, theta3 = delta_calcAngleYZ(x0*math.cos(2*math.pi/3) - y0*math.sin(2*math.pi/3), y0*math.cos(2*math.pi/3) + x0*math.sin(2*math.pi/3), z0)
        print(f"theta3: status={status}, angle={theta3}")
    print(f"Resultado: status={status}, theta1={theta1}, theta2={theta2}, theta3={theta3}")
    return status, theta1, theta2, theta3
# Example usage
if __name__ == "__main__":
    x0 = 100  # Example value for x0
    y0 = 100  # Example value for y0
    z0 = -300  # Example value for z0
    status, theta1, theta2, theta3 = delta_calcInverse(x0, y0, z0)
    if status == 0:
        print("Theta1:", theta1)
        print("Theta2:", theta2)
        print("Theta3:", theta3)
    else:
        print("Non-existing position")
