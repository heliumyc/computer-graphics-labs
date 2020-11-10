from scipy.spatial import ConvexHull, convex_hull_plot_2d
import numpy as np
mapper = lambda arr: list(map(lambda x: float(x), arr))
with open('./result.xyz', 'r') as f:
    f.readline()
    points = [mapper(x.split()[:-1]) for x in f]

points = np.array(points)
print(points)
import matplotlib.pyplot as plt
plt.plot(points[:,0], points[:,1], 'o')
plt.show()

