import numpy as np
import matplotlib.pyplot as plt
import cv2

slice = np.load("esdf_slice_test.npy")
# plt.imshow(slice)
# plt.show()

grad = np.gradient(slice)
grad = np.abs(grad[0]) + np.abs(grad[1])

def get_grad_value(size, i, j):
    plusX = 0
    minusX = 0
    plusY = 0
    minusY = 0

    for x in range(1, size + 1):
        plusX += slice[i + x, j]
        minusX += slice[i - x, j]
        plusY += slice[i, j + x]
        minusY += slice[i, j - x]
    return abs(plusX - minusX) + abs(plusY - minusY)

manual_grad = np.zeros_like(slice)
bigger_grad = np.zeros_like(slice)
size = 4
for i in range(size, slice.shape[0] - size):
    for j in range(size, slice.shape[1] - size):
        bigger_grad[i, j] = get_grad_value(size, i, j)
        # bigger_grad[i, j] = np.abs((slice[i + 2, j] + slice[i + 1, j]) - (slice[i - 1, j] + slice[i - 2, j])) + np.abs((slice[i, j + 2] + slice[i, j + 1]) - (slice[i, j - 1] + slice[i, j - 2]))
        # manual_grad[i, j] = np.abs(slice[i + 1, j] - slice[i - 1, j]) + np.abs(slice[i, j + 1] - slice[i, j - 1])

# print(np.sum(manual_grad))
# print(np.sum(grad))
# print(np.sum(manual_grad - 2 *grad))
plt.imshow(slice)
plt.show()
plt.imshow(bigger_grad)
# plt.imshow(manual_grad - grad)
plt.show()

plt.imshow(grad)
plt.show()

# kernal = np.array([[1, 2, 1], [0, 0, 0], [-1, -2, -1]])

# grad = np.convolve(slice, [1, 2, 1], mode='same')
# grad = np.abs(grad[0]) + np.abs(grad[1])
# grad = cv2.Sobel(grad, cv2.CV_64F, 1, 1, ksize=5)

# plt.imshow(grad * 25)
# plt.imshow(np.abs(grad[0]) + np.abs(grad[1]))
# plt.show() 
