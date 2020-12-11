import cv2 
  

video_files = ['video_linear_progression.mp4',
    'video_lateral_undulation.mp4', 
    'video_rolling.mp4',
    'video_rotation_about_axis.mp4',
    'video_sidewinding.mp4',
    'video_caterpillar.mp4']

titles = ['Linear progession', 'Lateral undulation', 'Rolling', 'Rotation about axis', 'Sidewinding', 'Caterpillar']

video = cv2.VideoCapture('../video/'+video_files[0])
count = 0
    
if (video.isOpened() == False):  
    print("Error reading video file") 
   
frame_width = 1500 
frame_height = 800
   
size = (frame_width, frame_height) 
print(size)
   


result = cv2.VideoWriter('../video/all_gaits.avi',  cv2.VideoWriter_fourcc(*'MJPG'), 20, size) 

while(True): 
    ret, frame = video.read() 
  
    if ret == True:  
        frame = cv2.resize(frame, size)

        position = (10,50)
        msg = titles[count]
        cv2.putText(
            frame, #numpy array on which text is written
            msg, #text
            position, #position at which writing has to start
            cv2.FONT_HERSHEY_SIMPLEX, #font family
            1, #font size
            (255, 255, 255), #(209, 80, 0, 255), #font color
            3) #font stroke 
        
        
        
        result.write(frame) 
  
        cv2.imshow('Frame', frame) 
  
        # Press S on keyboard  
        # to stop the process 
        if cv2.waitKey(1) & 0xFF == ord('s'): 
            break
  
    # Break the loop 
    else:
        count += 1
        if count < len(video_files):
            video = cv2.VideoCapture('../video/'+video_files[count])
        else:
            break
  



video.release() 
result.release() 

cv2.destroyAllWindows() 
   
print("The video was successfully saved")