import cv2
import numpy as np

def nothing(x):
    pass

cap = cv2.VideoCapture(0)

cv2.namedWindow("Trackbars")

# HSV sliders with your default values
cv2.createTrackbar("H_min","Trackbars",100,179,nothing)
cv2.createTrackbar("H_max","Trackbars",113,179,nothing)

cv2.createTrackbar("S_min","Trackbars",118,255,nothing)
cv2.createTrackbar("S_max","Trackbars",230,255,nothing)

cv2.createTrackbar("V_min","Trackbars",113,255,nothing)
cv2.createTrackbar("V_max","Trackbars",255,255,nothing)

# Noise filtering sliders
cv2.createTrackbar("Blur","Trackbars",20,20,nothing)
cv2.createTrackbar("Erode","Trackbars",2,10,nothing)
cv2.createTrackbar("Dilate","Trackbars",1,10,nothing)

while True:

    ret, frame = cap.read()
    if not ret:
        break

    # Read slider values
    h_min = cv2.getTrackbarPos("H_min","Trackbars")
    h_max = cv2.getTrackbarPos("H_max","Trackbars")

    s_min = cv2.getTrackbarPos("S_min","Trackbars")
    s_max = cv2.getTrackbarPos("S_max","Trackbars")

    v_min = cv2.getTrackbarPos("V_min","Trackbars")
    v_max = cv2.getTrackbarPos("V_max","Trackbars")

    blur_val = cv2.getTrackbarPos("Blur","Trackbars")
    erode_val = cv2.getTrackbarPos("Erode","Trackbars")
    dilate_val = cv2.getTrackbarPos("Dilate","Trackbars")

    # Ensure blur is odd
    if blur_val % 2 == 0:
        blur_val += 1
    if blur_val < 1:
        blur_val = 1

    frame_blur = cv2.GaussianBlur(frame,(blur_val,blur_val),0)
    hsv = cv2.cvtColor(frame_blur, cv2.COLOR_BGR2HSV)

    lower = np.array([h_min,s_min,v_min])
    upper = np.array([h_max,s_max,v_max])

    mask = cv2.inRange(hsv, lower, upper)

    if erode_val > 0:
        mask = cv2.erode(mask,None,iterations=erode_val)

    if dilate_val > 0:
        mask = cv2.dilate(mask,None,iterations=dilate_val)

    contours,_ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    if contours:

        c = max(contours,key=cv2.contourArea)

        if cv2.contourArea(c) > 500:

            M = cv2.moments(c)

            if M["m00"] != 0:
                cx = int(M["m10"]/M["m00"])
                cy = int(M["m01"]/M["m00"])

                cv2.circle(frame,(cx,cy),6,(0,255,0),-1)
                cv2.drawContours(frame,[c],-1,(255,0,0),2)

                cv2.putText(frame,f"Center: {cx},{cy}",
                            (cx+10,cy),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,(0,255,0),2)

    cv2.imshow("Frame",frame)
    cv2.imshow("Mask",mask)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
