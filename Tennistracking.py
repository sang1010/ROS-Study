#!/usr/bin/env python
import cv2
import numpy as np
import cv2
import time

def get_contour_center(contour):    
    M = cv2.moments(contour)
    cx=-1
    cy=-1
    if (M['m00']!=0):
        cx= int(M['m10']/M['m00'])
        cy= int(M['m01']/M['m00'])
    return cx, cy


def read_video(video_name):
    video_capture = cv2.VideoCapture(video_name)

    while(True):
        ret, frame = video_capture.read()
        #show RGB image
        cv2.imshow("Frame",frame)

        #convert RGB image into HSV image
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        cv2.imshow("hsv image",hsv_frame)

        #find the upper and lower bounds of the yellow color (tennis ball)
        yellowLower =(30, 150, 100)
        yellowUpper = (50, 255, 255)

        #define a mask using the lower and upper bounds of the yellow color 
        mask_frame = cv2.inRange(hsv_frame, yellowLower, yellowUpper)
        cv2.imshow("mask image", mask_frame)
        #draw grayscale image through Color filtering


        #####################################################

        _, contours, hierarchy = cv2.findContours(mask_frame.copy(), 
                                            cv2.RETR_EXTERNAL,
	                                        cv2.CHAIN_APPROX_SIMPLE)
 


        ####################################################        
        for c in contours:
            area = cv2.contourArea(c)
            perimeter= cv2.arcLength(c, True)
            ccc = cv2.minEnclosingCircle(c)
            ((x, y), radius) = ccc
            if (area>3000):
                # cv2.drawContours(frame, [c], -1, (255,0,0), 1)
                cx, cy = get_contour_center(c)
                cv2.circle(frame, (cx,cy),(int)(radius),(0,0,255),3)
                print ("Area: {}, Perimeter: {}".format(area, perimeter))
        print ("number of contours: {}".format(len(contours)))
        # print(ccc)
        # time.sleep(10)
        cv2.imshow("RGB Image Contours",frame)
        #Draw contour circle in RGB image




        if cv2.waitKey(25) & 0xFF == ord('q'):
            break

    video_capture.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    read_video('video/tennis-ball-video.mp4')

