## Snake Robot - ROS Controllers


### Instructions to run the code:
- Clone the repo using: $ git clone https://github.com/anubhavparas/snake_robot.git
- $ cd snake_robot/snakebot_ws
- $ catkin_make
- $ source devel/setup.bash
- To spawn the controllers and the model in gazebo: $ roslaunch snakebot snakebot.launch
- To run the gait simulation, for example: to move the snake in the simple linear forward motion:
  - Open a new terminal
  - Change the directory to /snakebot_ws
  - $ source devel/setup.bash
  - $ rosrun snakebot linear_progression.py
### Results: Output video
- [All the snake gaits](https://drive.google.com/file/d/1BfiJ1PDn6ounzhUILLyYK5kYYPiaXl7P/view?usp=sharing)
- [Trannsformer gait](https://drive.google.com/file/d/1lpOsV6T_p5WpRXYhA7TPNdCAyq_wULQ6/view?usp=sharing)



