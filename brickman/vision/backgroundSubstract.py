import numpy as np
import cv2
import os

cap = cv2.VideoCapture(0)

fgbg = cv2.createBackgroundSubtractorMOG2()

def detect_target(image):
    #Detect the center of the target circle (Colour: [200,200,200])
    #SAME AS DETECT_BLUE JUST WITH DIFFERENT COLOUR LIMITS
    mask = cv2.inRange(image, (0,0,0),(10,10,10))
    kernel = np.ones((5,5),np.uint8)
    mask = cv2.dilate(mask,kernel,iterations=3)
    M = cv2.moments(mask)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])

    return (cx,cy)



#main method
def main():
    frame_num = 0
    while(cap.isOpened()):
        ret, frame = cap.read()
        # update frame number
        frame_num = frame_num + 1
        history = 50
        fgmask = fgbg.apply(frame, learningRate=1.0/history)
        #fgmask = fgbg.apply(frame)
        #print(fgmask)



        # write image
        cv2.imwrite(str(frame_num) + '.png',frame)
        img = cv2.imread(str(frame_num) + '.png',0)






        ret,thresh = cv2.threshold(img,127,255,0)
        _,contours,_ = cv2.findContours(thresh, 1, 2)

        cnt = contours[0]
        M = cv2.moments(cnt)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
        else:
            cx, cy = 0, 0
        x,y,w,h = cv2.boundingRect(cnt)
        img = cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
        (x,y),radius = cv2.minEnclosingCircle(cnt)
        center = (int(x),int(y))
        radius = int(radius)
        img = cv2.circle(img,center,radius,(0,255,0),2)
        cv2.imshow('img', img)




        # get the position of object

        # ret,thresh = cv2.threshold(img,127,255,0)
        # _,contours,_ = cv2.findContours(thresh, 1, 2)
        # if len(contours) != 0:
        #     cnt = contours[0]
        #     M = cv2.moments(cnt)
        #
        #
        #
        #
        #     if M["m00"] != 0:
        #         cx = int(M["m10"] / M["m00"])
        #         cy = int(M["m01"] / M["m00"])
        #     else:
        #         cx, cy = 0, 0
        #
        #
        #
        #     x,y,w,h = cv2.boundingRect(cnt)
        #     img = cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
        #
        #     #print(center_X)
        #     #print(' , ')
        #     #print(center_Y)
        #     cv2.imshow('img', img)

        os.remove(str(frame_num) + '.png')
        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # define range of blue color in HSV
        lower_white = np.array([110,50,50])
        upper_white = np.array([130,255,255])
        # Threshold the HSV image to get only white colors
        mask = cv2.inRange(hsv, lower_white, upper_white)
        #cv2.imshow('fgmask1', fgmask1)
        #targetX, targetY = detect_target(fgmask)
        #print(targetX + ' , ' + targetY)


        cv2.imshow('frame',fgmask)
        k = cv2.waitKey(30) & 0xff
        if k == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
