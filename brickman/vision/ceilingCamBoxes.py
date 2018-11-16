import numpy as np
import cv2
import os
import math
import paho.mqtt.client as mqtt
import json

class ceilingCam():
    def __init__(self, leftBound, rightBound, lowerBound, upperBound):
        self.home = (520,420)
        # define the coundry of the room
        self.leftBound = leftBound
        self.rightBound = rightBound
        self.lowerBound = lowerBound
        self.upperBound = upperBound
        self.boxRadius = 20
        self.robotMoving = False
        self.facingObject = False
        self.notRunning = True
        self.notRunningSlow = True
        self.notClose = True

    def robotStopped(self):
        self.robotMoving = False

    def dotproduct(self, v1, v2):
        return sum((a*b) for a, b in zip(v1, v2))

    def length(self, v):
        return math.sqrt(self.dotproduct(v, v))

    # calculate angle bweteen two vectors
    def getAngle(self, v1, v2):
        # given two vector, calculate angles in between
        # in degrees
        # add 0.00001 to fix division by zero
        return math.degrees(math.acos(self.dotproduct(v1, v2) / \
                ((self.length(v1)+0.000001) * self.length(v2))))

    # calculate angles given centers of front of car, back of car and targtet
    def calculateAngle(self, front, back, target):
        dx = front[0] - back[0]
        dy = front[1] - back[1]

        dx1 = target[0]- back[0]
        dy1 = target[1]- back[1]

        # decide which side is the target
        # when d is less than 0 means to the left of robot
        d = (target[0]-back[0])*(front[1]-back[1])\
            - (target[1]-back[1])*(front[0]-back[0])
        if d >= 0:
            # negative angle means to the left
            angle = - self.getAngle([dx,dy], [dx1,dy1])
        else:
            # right angle means to the right
            angle = self.getAngle([dx,dy], [dx1,dy1])

        return angle

    # check if the given position is within the boundray of a room
    def insideRoom(self, x, y):
        if x >= self.leftBound and x <= self.rightBound \
            and y >= self.lowerBound and y <= self.upperBound:
            return True
        else:
            return False

    # calculate distance between two positions(tuple)
    def getDistance(self, pt1, pt2):
        x1, y1 = pt1
        x2, y2 = pt2
        dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        return dist

    # detect red objects and return their positions
    # size parameter is to filter object to be detected
    def detectRed(self, img, size, box=False):
        # stores all object positions
        positions = []

        redLower_lo = (0, 80, 50)
        redUpper_lo = (10, 250, 250)

        redLower_hi = (170, 50, 50)
        redUpper_hi = (180, 255, 255)

        blurred = cv2.GaussianBlur(img, (15, 15), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        mask_lo = cv2.inRange(hsv, redLower_lo, redUpper_lo)
        mask_hi = cv2.inRange(hsv, redLower_hi, redUpper_hi)
        mask = mask_lo + mask_hi
        mask = cv2.erode(mask, None, iterations=1)
        mask = cv2.dilate(mask, None, iterations=1)
        ret,thresh = cv2.threshold(mask,127,255,0)
        im2,contours,hierarchy = cv2.findContours(thresh, 1, 2)

        for cnt in contours:
            M = cv2.moments(cnt)
            (x,y),radius = cv2.minEnclosingCircle(cnt)

            # only proceed if the area meets a minimum size
            if radius > size and radius < self.boxRadius and self.insideRoom(x,y)  and not box:
                center = (int(x),int(y))
                radius = int(radius)
                text = 'red ' +str(radius)
                cv2.putText(img, text,center,cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                positions.append((x,y))
            elif radius >= self.boxRadius and self.insideRoom and box:
                center = (int(x),int(y))
                radius = int(radius)
                text = 'red Box' +str(radius)
                cv2.putText(img, text,center,cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                positions.append((x,y))

        cv2.imshow('red',mask)
        return img, positions

    # detect blue objects and return their positions
    def detectBlue(self, img, size):
        # stores all object positions
        positions = []

        blueLower = (100,100,46)
        blueUpper = (124,255,255)

        blurred = cv2.GaussianBlur(img, (15, 15), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, blueLower, blueUpper)
        mask = cv2.erode(mask, None, iterations=1)
        mask = cv2.dilate(mask, None, iterations=1)
        ret,thresh = cv2.threshold(mask,127,255,0)
        im2,contours,hierarchy = cv2.findContours(thresh, 1, 2)

        for cnt in contours:
            M = cv2.moments(cnt)
            (x,y),radius = cv2.minEnclosingCircle(cnt)

            # only proceed if the area meets a minimum size
            if radius > size and self.insideRoom(x,y):
                center = (int(x),int(y))
                radius = int(radius)
                text = 'blue' + str(radius)
                cv2.putText(img, text,center,cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                positions.append((x,y))

            elif radius >= self.boxRadius and self.insideRoom:
                center = (int(x),int(y))
                radius = int(radius)
                text = 'blue Box' +str(radius)
                cv2.putText(img, text,center,cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                positions.append((x,y))

        #cv2.imshow('blue',mask)
        return img, positions

    # detect green objects and return their positions
    def detectGreen(self, img, size):
        # stores all object positions
        positions = []

        greenLower = (30,100,46)
        greenUpper = (85,255,255)

        blurred = cv2.GaussianBlur(img, (15, 15), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=1)
        mask = cv2.dilate(mask, None, iterations=1)
        ret,thresh = cv2.threshold(mask,127,255,0)
        im2,contours,hierarchy = cv2.findContours(thresh, 1, 2)

        for cnt in contours:
            M = cv2.moments(cnt)
            (x,y),radius = cv2.minEnclosingCircle(cnt)

            # only proceed if the area meets a minimum size
            if radius > size and self.insideRoom(x,y):
                center = (int(x),int(y))
                radius = int(radius)
                text = 'green' + str(radius)
                cv2.putText(img, text,center,cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                positions.append((x,y))

            elif radius >= self.boxRadius and self.insideRoom:
                center = (int(x),int(y))
                radius = int(radius)
                text = 'green Box' +str(radius)
                cv2.putText(img, text,center,cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                positions.append((x,y))

        #cv2.imshow('green',mask)
        return img, positions

    # detect yellow objects and return their positions
    def detectYellow(self, img, size):
        # stores all object positions
        positions = []

        yellowLower = (22,40,0)
        yellowUpper = (81,255,255)

        blurred = cv2.GaussianBlur(img, (15, 15), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, yellowLower, yellowUpper)
        mask = cv2.erode(mask, None, iterations=1)
        mask = cv2.dilate(mask, None, iterations=1)
        ret,thresh = cv2.threshold(mask,127,255,0)
        im2,contours,hierarchy = cv2.findContours(thresh, 1, 2)

        for cnt in contours:
            M = cv2.moments(cnt)
            (x,y),radius = cv2.minEnclosingCircle(cnt)

            # only proceed if the area meets a minimum size
            if radius > size and self.insideRoom(x,y):
                center = (int(x),int(y))
                radius = int(radius)
                text = 'yellow' + str(radius)
                cv2.putText(img, text,center,cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                positions.append((x,y))

            elif radius >= self.boxRadius and self.insideRoom:
                center = (int(x),int(y))
                radius = int(radius)
                text = 'yellow Box' +str(radius)
                cv2.putText(img, text,center,cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                positions.append((x,y))

        #cv2.imshow('yellow',mask)
        return img, positions

    # detect and return the position(center) of front and back of robot
    def detectRobot(self, img):
        # real black mask below
        blackLower = (0, 0, 0)
        blackUpper = (218, 255, 80)
        # actually green mask below
        # blackLower = (30,100,46)
        # blackUpper = (85,255,255)

        blurred = cv2.GaussianBlur(img, (15, 15), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, blackLower, blackUpper)
        #mask = cv2.inRange(hsv, pinkLower, pinkUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        #mask = mask_lo + mask_hi
        ret,thresh = cv2.threshold(mask,127,255,0)

        im2,contours,hierarchy = cv2.findContours(thresh, 1, 2)
        # cnts stores cnt inside room boundray with specific size
        cnts = []
        for cnt in contours:
            (x,y),radius = cv2.minEnclosingCircle(cnt)
            if radius > 5 and radius < 150 and self.insideRoom(x,y):
                cnts.append(cnt)

        if len(cnts) > 1:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            centerSquare = None
            centerCircle = None
            angle = None

            cntFront = sorted(cnts, key=cv2.contourArea)[-1]
            cntBack = sorted(cnts, key=cv2.contourArea)[-2]

            (x,y),radius = cv2.minEnclosingCircle(cntFront)
            (x1,y1),radius1 = cv2.minEnclosingCircle(cntBack)

            center_front = (int(x),int(y))
            radius_front = int(radius)
            cv2.circle(img,center_front,radius_front,(0,255,0),2)

            center_back = (int(x1),int(y1))
            radius_back = int(radius1)
            cv2.circle(img,center_back,radius_back,(255,0,0),2)

            cv2.putText(img, 'front',center_front,cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.putText(img, 'back',center_back,cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.imshow('robot',mask)
            return img, center_front, center_back

        return None, None, None

    # put it all togetherdistance_object
    def go(self, img):
        # get all red object positions with size greater 10
        _, positions_red = self.detectRed(img, 9)
        _, red_boxes = self.detectRed(img, 9, True)
        # get all blue object positions
        _, positions_blue = self.detectBlue(img, 9)

        # get all green object positions
        _, positions_green = self.detectGreen(img, 9)

        # get all yellow object positions
        _, positions_yellow = self.detectYellow(img, 9)

        # get all ywlloe object positions
        # _, positions_yellow = self.detectYellow(img)

        # merge all positions
        positions = positions_red

        # get robot data
        _, center_front, center_back = self.detectRobot(img)


        if center_front is not None and len(positions) >= 1:
            targets = []
            for p in positions:
                dist = self.getDistance(p, center_front)
                targets.append((p,dist))

            # sort targets based on distance
            sorted_targets = sorted(targets, key=lambda x: x[1])
            first_target_center = sorted_targets[0][0]

            red_home = self.home
            
            if red_boxes:
                red_home = red_boxes[0]


            angle_obj = self.calculateAngle(center_front, center_back, first_target_center)
            angle_home = self.calculateAngle(center_front, center_back, self.home)
            distance_object = self.getDistance(center_front,first_target_center)
            # round angles to two decimal places
            angle_obj = math.ceil(angle_obj*100)/100
            angle_home = math.ceil(angle_home*100)/100
            distance_object = round(distance_object,5)
            distance_home = self.getDistance(center_front,self.home)
            distance_home = round(distance_home,5)

            # Find distance and angle from red home
            angle_redHome = self.calculateAngle(center_front, center_back, red_home)
            angle_redHome = math.ceil(angle_redHome*100)/100
            distance_redHome = self.getDistance(center_front,red_home)
            distance_redHome = round(distance_redHome,5)

            com ='''
            if not self.robotMoving:
                send_msg = {
                    'command': 'turn',
                    'payload':{
                    'degrees': angle_obj,
                    }
                }

                client.publish("json", payload=json.dumps(send_msg))
                self.robotMoving = True
                #self.notRunningSlow = True

            if distance_object < 100 and self.notRunningSlow:
                send_msg = {
                    'command': 'run_straight',
                    'payload':{
                    'speed': 50,
                    }
                }
                turn_msg = {
                    'command': 'turn',
                    'payload':{
                    'degrees': angle_obj,
                    }
                }
                client.publish("json", payload=json.dumps(turn_msg))
                client.publish("json", payload=json.dumps(send_msg))
                #self.robotMoving = False
                self.notRunningSlow = False

            if distance_object < 35 and self.notClose and self.facingObject:
                turn_msg = {
                    'command': 'turn',
                    'payload':{
                    'degrees': angle_obj,
                    }
                }
                send_msg = {
                    'command': 'close_claw'
                }
                turn_home = {
                    'command': 'turn',
                    'payload':{
                    'degrees': angle_home,
                    }
                }invision
                run_msg = {
                    'command': 'run_straight',
                    'payload':{
                    'speed': 600,
                    }
                }
                client.publish("json", payload=json.dumps(turn_msg))
                client.publish("json", payload=json.dumps(send_msg))
                client.publish("json", payload=json.dumps(turn_home))
                client.publish("json", payload=json.dumps(run_msg))
                self.notClose = False
                self.notRunningSlow = False

            if abs(angle_obj) < 5 and distance_object >= 100 and self.notRunning:
                self.facingObject = True
                send_msg = {
                    'command': 'run_straight',
                    'payload':{
                    'speed': 700,
                    }
                }
                client.publish("json", payload=json.dumps(send_msg))
                self.notRunning = False

            if distance_home < 75 and not self.notClose:
                send_msg = {
                    'command': 'stop',
                }
                client.publish("json", payload=json.dumps(send_msg))
                '''

            cv2.putText(img, "angle from object: {}, angle from red home: {}, distance from object: {}"\
                        .format(angle_obj,angle_redHome,distance_object),
                (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                0.45, (0, 0, 0), 1)


if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    comm ='''
    client = mqtt.Client(clean_session=True)
    client.connect("192.168.105.117", 1883,60)
    client.subscribe([("finished",2)])
    '''
    camera = ceilingCam(40,550,80,450)

    comm2 = '''
    def on_message(client, userdata, msg):
        print("MESSAGE RECEIVED")
        if msg.topic == 'finished':
            camera.robotStopped()

    client.on_message = on_message
    '''
    camera = ceilingCam(40,550,80,450)

    frame_num = 0

    # while(not camera.facingObject):
    while(1):
        ret, frame = cap.read()
        frame_num = frame_num + 1


        cv2.imwrite(str(frame_num) + '.jpg',frame)
        img = cv2.imread(str(frame_num) + '.jpg')
        camera.go(img)

        # show home on img
        cv2.putText(img, 'Home',camera.home,cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.imshow('img',img)

        os.remove(str(frame_num) + '.jpg')
        k = cv2.waitKey(30) & 0xff
        if k == 27:
            break

    cap.release()
    cv2.destroyAllWindows()
