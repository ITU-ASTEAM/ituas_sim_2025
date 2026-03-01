import numpy as np

data = np.load("costmap_20260228_194329_points.npy")
print(data.shape)
print(data.dtype)
print(data[:10])
