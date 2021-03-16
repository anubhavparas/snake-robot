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

[![All the snake gaits](https://user-images.githubusercontent.com/32901101/111253024-ca6d4e00-85e8-11eb-8b49-166eb2cf8421.PNG)](https://drive.google.com/file/d/1BfiJ1PDn6ounzhUILLyYK5kYYPiaXl7P/view?usp=sharing)

[![Transformer gait](https://user-images.githubusercontent.com/32901101/111253242-3bad0100-85e9-11eb-8c0c-89df66aeea76.PNG)](https://drive.google.com/file/d/1lpOsV6T_p5WpRXYhA7TPNdCAyq_wULQ6/view?usp=sharing)


### References:
- [https://en.wikipedia.org/wiki/Gait](https://en.wikipedia.org/wiki/Gait)
- [https://en.wikipedia.org/wiki/Sidewinding](https://en.wikipedia.org/wiki/Sidewinding)
- [https://www.ivlabs.in/uploads/1/3/0/6/130645069/rebis.pdf](https://www.ivlabs.in/uploads/1/3/0/6/130645069/rebis.pdf)
- [http://biorobotics.ri.cmu.edu/projects/modsnake/gaits/gaits.html](http://biorobotics.ri.cmu.edu/projects/modsnake/gaits/gaits.html)
- [https://userweb.ucs.louisiana.edu/~brm2286/locomotn.htm](https://userweb.ucs.louisiana.edu/~brm2286/locomotn.htm)
- [http://biorobotics.ri.cmu.edu/research/gaitResearch.php](http://biorobotics.ri.cmu.edu/research/gaitResearch.php)



