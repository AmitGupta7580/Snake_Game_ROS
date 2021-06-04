# Snake Game (ROS)

ROS package of snake Game implemented in pygame

# Preview
<div>
  <img src="https://github.com/AmitGupta7580/Snake_Game_ROS/blob/master/snake_game/Snake%20Game-1.png" alt="Snake and Food" width="400" height="500">
  <img src="https://github.com/AmitGupta7580/Snake_Game_ROS/blob/master/snake_game/Snake%20Game-2.png" alt="Snake and Food" width="400" height="500">
</div>

# ROS Nodes
<ul>
  <li>server.py (pulishes the state of snake world and subscribe the command to move snake)</li> 
  <li>client.py (subscribe to the state of snake world and publishes the direction of the snake to move)</li>
</ul>

# ROS Message
<ul>
  <li>position (x: x-corrdinate, y: y-corrdinate)</li> 
</ul>

# ROS Topics
<ul>
  <li>/snake_game/score (Type : std_msgs/Int32) (current score of the agent)</li> 
  <li>/snake_game/food (Type : snake_game.msg/position) (current position of the food)</li>
  <li>/snake_game/snake (Type : snake_game.msg/position) (current position of the snake_head)</li>
  <li>/snake_game/move (Type : std_msgs/String) (direction in which snake has to be moved)</li>
  <ul>
    <li> Options : 'Left', 'Right', 'Back', 'Front'</li>
    <li> If snake cann't move it raise an Exception and start new episode with 0 score </li>
  </ul>
</ul>

# Future Work

Try to implement Reinforcement Learning on this game.
