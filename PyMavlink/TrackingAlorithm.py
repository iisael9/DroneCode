""" Tracking Algorithm 

State Machine Visualization

__Commander

    if battery < 50 % return Land
    Have drone fly out to a location and then scan and start to search w tflite for 
    the other drone with out __Detection and then reposition with our __Position_Calculations
    We use pymavlink to control the drone
        - Take_Off, Land, Go_To



__Detection(%, L, W, D, X, Y)    (X,Y) Centtroid 
    need to get information of where it is positioned on the screen
    


__Position_Calculations
    with information of where it is on the screen we reposition the drone to center 
    the other drone by (Yaw) Horizontal Rotation done by __Reposition_Center_Detected_Drone
    When detecting can we smooth out the info or only use the info when the graph is
    flat

    __Reposition_Center_Out_Detected_Drone(Confidence% > 50%, X, Y, GPS)

    Does a python Lib have a way to pull the X, Y on the screen




"""