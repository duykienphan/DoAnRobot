import numpy as np

R = np.ones((8, 8)) * 10
R2 = np.array([[1, 2, 3, 4, 5, 6, 7, 8],
              [8, 7, 6, 5, 4, 3, 2, 1]])

print(np.matmul(R2.T, R2))