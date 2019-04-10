def laneDetection(image):
    # Read image
    # image = cv2.imread("camino_art_4.png")
    im_in = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    # cropping image
    third_y = int(im_in.shape[0]/2)
    width = int(im_in.shape[1])
    height = int(im_in.shape[0])

    cropped_image = im_in
    img = image
    #cropped_image = im_in[third_y:height,0:width]
    #img = image[third_y:height,0:width]

    h_img = int(img.shape[0])

    # Gaussian blur
    blurred_image_1 = cv2.GaussianBlur(cropped_image, (15, 15), 0)

    # Bilateral filter
    blurred_image_2 = cv2.bilateralFilter(cropped_image,15,55,55)

    # Threshold
    #th, im_th = cv2.threshold(blurred_image_2, 200, 255, cv2.THRESH_BINARY)
    th, im_th = cv2.threshold(blurred_image_1, 200, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # Morphological transformantion
    # closing
    kernel = np.ones((9,9),np.uint8)
    closing = cv2.morphologyEx(im_th, cv2.MORPH_CLOSE, kernel, iterations = 1)

    # Ffind contours
    im2, contours, hierarchy = cv2.findContours(closing,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

    #ordered contours
    ordered_contours = sorted(contours, key = cv2.contourArea, reverse = True)

    cx_array, cy_array, centroids = my.obtainCentroid(ordered_contours, 50)

    # Draw centroids
    for x in ordered_contours:
        if cv2.contourArea(x) > 50:
            cv2.drawContours(img, x, -1, (0,0,255), 3)

    # Sorting centroids
    centroids_sorted = sorted(centroids, key = operator.itemgetter(0), reverse=True)

    # Preparing centroids for linear regresion
    x = []
    y = []
    for i in centroids_sorted:
        x.append(i[0])
        y.append(i[1])

    # Center point of continuous line
    x_c = centroids_sorted[0][0]
    y_c = centroids_sorted[0][1]

    # Exclude most right centroid (continuous line)
    x = x[1:]
    y = y[1:]

    # linear regresion procedure
    m, b, x_d, y_d, x_m, y_m = my.linearRegression(x, y, h_img)

    y_t = [0, h_img]
    y_t = np.array(y_t, dtype=np.float64)
    x_t = (y_t - b)/m

    y_d = int(sum(y_t)/len(y_t))
    x_d = int((y_d - b)/m)

    # Obtain car center
    x_car, y_car = my.obtainCarCenter(x_d, y_d, x_c, y_c)

    # Draw obtained lines and point
    cv2.line(img, (int(x_m[0]), int(y_m[0])), (int(x_m[-1]), int(y_m[-1])), (255, 0, 0), thickness = 4)
    cv2.line(img, (x_d, y_d), (x_c, y_c), (0, 200, 200), thickness=4)
    cv2.circle(img,(x_car, y_car), 8, (0, 255, 189), 3)

    # Final image with blobs and centroids
    # cv2.imshow("lines", img)
    return img
