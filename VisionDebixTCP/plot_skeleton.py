"""
===========
Skeletonize
===========

Skeletonization reduces binary objects to 1 pixel wide representations. This
can be useful for feature extraction, and/or representing an object's topology.

``skeletonize`` works by making successive passes of the image. On each pass,
border pixels are identified and removed on the condition that they do not
break the connectivity of the corresponding object.
"""



######################################################################
# **Morphological thinning**
#
# Morphological thinning, implemented in the `thin` function, works on the
# same principle as `skeletonize`: remove pixels from the borders at each
# iteration until none can be removed without altering the connectivity. The
# different rules of removal can speed up skeletonization and result in
# different final skeletons.
#
# The `thin` function also takes an optional `max_num_iter` keyword argument to
# limit the number of thinning iterations, and thus produce a relatively
# thicker skeleton.
from skimage.morphology import thin
from skimage import data
import matplotlib.pyplot as plt
from skimage.util import invert
import cv2

image = cv2.imread("ogImage.jpg")
# Gray
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
# Gaussian blur to reduce noise
blurred_image = cv2.GaussianBlur(gray, (9, 9), 0)
# Otsu Threshold
_, otsu_thresh = cv2.threshold(blurred_image, 100, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

# Morphology for noise reduction
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
morph_image = cv2.morphologyEx(otsu_thresh, cv2.MORPH_CLOSE, kernel)

inverted_image = invert(morph_image)
thinned = thin(inverted_image)

fig, axes = plt.subplots(2, 2, figsize=(8, 8), sharex=True, sharey=True)
ax = axes.ravel()

ax[0].imshow(image)
ax[0].set_title('original')
ax[0].axis('off')

ax[1].imshow(otsu_thresh, cmap=plt.cm.gray)
ax[1].set_title('Otsu\'s Thresholding')
ax[1].axis('off')

ax[2].imshow(morph_image, cmap=plt.cm.gray)
ax[2].set_title('morph_image')
ax[2].axis('off')

ax[3].imshow(thinned, cmap=plt.cm.gray)
ax[3].set_title('thinned')
ax[3].axis('off')

fig.tight_layout()
plt.show()
