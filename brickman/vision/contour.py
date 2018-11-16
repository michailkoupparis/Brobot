import numpy as np
import cv2
import os

cap = cv2.VideoCapture(0)
fgbg = cv2.createBackgroundSubtractorMOG2()

frame_num = 0
while(cap.isOpened()):
    ret, frame = cap.read()
    # update frame number
    frame_num = frame_num + 1
    history = 50
    fgmask = fgbg.apply(frame, learningRate=1.0/history)
    cv2.imwrite(str(frame_num) + '.jpg',fgmask)


    img = cv2.imread(str(frame_num) + '.jpg',0)
    ret,thresh = cv2.threshold(img,127,255,0)



    im2,contours,hierarchy = cv2.findContours(thresh, 1, 2)
    if len(contours) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        cnt = max(contours, key=cv2.contourArea)
        M = cv2.moments(cnt)
        area = cv2.contourArea(cnt)
        (x,y),radius = cv2.minEnclosingCircle(cnt)
        # only proceed if the area meets a minimum size
        if radius > 30:
            #cx = int(M['m10']/M['m00'])
            #cy = int(M['m01']/M['m00'])

            # rect = cv2.minAreaRect(cnt)
            # box = cv2.boxPoints(rect)
            # box = np.int0(box)
            # cv2.drawContours(frame,[box],0,(0,255,0),2)


            center = (int(x),int(y))
            radius = int(radius)
            print(center)
            height, width = frame.shape[:2]

            cv2.circle(frame,center,radius,(0,255,0),2)
            # circle is to the left of middle of image
            if center[0] < width/2:
                cv2.putText(frame, 'left',center,cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            # circle is at the right side
            else:
                cv2.putText(frame, 'right',center,cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    cv2.imshow('frame',frame)
    os.remove(str(frame_num) + '.jpg')
    k = cv2.waitKey(30) & 0xff
    if k == 27:
        break

cap.release()
cv2.destroyAllWindows()
