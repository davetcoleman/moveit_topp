import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import PPoly, splrep, splev

# Data points
x = [0, 1, 2, 3, 4, 5]
y = [0, 1, 4, 0, 2, 2]

# Fit B-spline. This is a bunch of cubic polinomials that blend well (C^2)
# Calls fortran's FITPACK under the hood
tck = splrep(x, y)
# Make piecewise polynomial from the B-spline
pp = PPoly.from_spline(tck)


# Some plotting to show how the coefficients are stored in the PPoly.
# Basically columns in pp.c are coefficients of [x^3, x^2, x, 1]
# and pp.x contains the bounds for each polynomial. The complete thing
# consists of evaluating the polynomial in pp.c[:, i] in the range pp.x[i:i+1]

xx = np.linspace(0, max(x))
yy = splev(xx, tck)  # Also evaluates derivatives by setting der=desired order

lw = 20
for i in range(len(pp.c.T)):
    plt.plot(xx, np.poly1d(pp.c[:, i])(xx-pp.x[i]), 'k', alpha=0.3, lw=lw)
    lw -= 2
plt.ylim(-1, 5)
plt.plot(xx, yy)
plt.show()
