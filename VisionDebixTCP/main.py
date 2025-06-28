import cv2
import numpy as np
from scipy.interpolate import splprep, splev

img = cv2.imread("lines5.jpeg")
img = cv2.resize(img, (512,512))
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
blur = cv2.GaussianBlur(gray,(5,5),2)
th3 = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 11)
#ret, th3 = cv2.threshold(blur, 225, 255, cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU) 

# Define a kernel for morphological operations
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

# Apply erosion to remove small noise
eroded = cv2.erode(th3, kernel, iterations=1)

# Apply dilation to restore the size of the main lines
cleaned = cv2.dilate(eroded, kernel, iterations=1)

# Apply Canny edge detection
edges = cv2.Canny(cleaned, 50, 150) 

#np.set_printoptions(threshold=np.inf, linewidth=np.inf)
#print(edges)

cv2.imshow('edges',edges)
height, width = img.shape[:2]

# Calculate bottom-left and bottom-right corners
bottom_left = (height - 1, 0)
bottom_right = (height - 1, width - 1)
cv2.floodFill(edges, mask=None, seedPoint=bottom_left, newVal=(255))

cv2.floodFill(edges, mask=None, seedPoint=bottom_left, newVal=(0))

cv2.floodFill(edges, mask=None, seedPoint=bottom_right, newVal=(255))

cv2.floodFill(edges, mask=None, seedPoint=bottom_right, newVal=(0))

cv2.imshow("floodFilled", edges)
''''''


lines = cv2.HoughLinesP(edges, 1,np.pi/180, 1, maxLineGap=30)
print(len(lines))
for i in range(len(lines)):
    line = lines[i]
#for line in lines:
    x1,y1,x2,y2 = line[0]
    cv2.line(img, (x1,y1),(x2,y2),(0,255,0),2)
cv2.imshow('Image'+str(i),img)
# Create a blank image
'''image = np.zeros((512, 512), dtype=np.uint8)

# Draw vectors on the image
for line in lines:
    x1,y1,x2,y2 = line[0]
    cv2.line(image, (x1,y1),(x2,y2),(255,255,255),3)

# Apply morphological operations (dilate and erode)
kernel = np.ones((5, 5), np.uint8)
dilated_image = cv2.dilate(image, kernel, iterations=1)
eroded_image = cv2.erode(dilated_image, kernel, iterations=1)

# Find contours in the processed image
contours, _ = cv2.findContours(eroded_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Draw the detected contours (merged lines)
result_image = np.zeros_like(image)
for contour in contours:
    # Convert contour to numpy array
    contour = contour.reshape((-1, 2))
    
    # Fit a spline to the contour points
    tck, u = splprep(contour.T, u=None, s=0.0, per=1)
    smooth_points = splev(np.linspace(0, 1, 100), tck)
    smooth_points = np.array(smooth_points, dtype=np.int32).T
    
    # Draw the smoothed contour
    cv2.polylines(result_image, [smooth_points], False, 255, 2)  # Draw in white with thickness 2

# Display the results
cv2.imshow('Original Image', image)
cv2.imshow('Merged Lines', result_image)
'''
#print(lines)
#cv2.imshow('Image',img)
#cv2.imshow('ImageGray',gray)
#cv2.imshow('Blur',blurred_image)
#cv2.imshow('ImageEdges',edges)
cv2.waitKey(0)
cv2.destroyAllWindows()