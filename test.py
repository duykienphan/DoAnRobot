import numpy as np

R = np.array([[1, 2, 3, 4, 5, 6, 7, 8],
              [8, 7, 6, 5, 4, 3, 2, 1]])
R2 = np.array([[1, 2, 3, 4, 5, 6, 7, 8],
              [8, 7, 6, 5, 4, 3, 2, 1]])
R3 = np.matmul(R.T, R2)
print(2*R3)