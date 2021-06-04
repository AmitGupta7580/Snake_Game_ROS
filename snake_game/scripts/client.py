#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Int32
from snake_game.msg import position

class SnakeClient:
  def __init__(self):

    # Initializing the ros node
    rospy.init_node('snake_client', anonymous=True)

    # Initializing the Subscriber
    rospy.Subscriber("/snake_game/score", Int32, self.score_callback)
    rospy.Subscriber("/snake_game/food", position, self.food_callback)
    rospy.Subscriber("/snake_game/snake", position, self.snake_callback)

  def food_callback(self, data):
    # position of food
    rospy.loginfo("Food : ({}, {})".format(data.x, data.y))

  def snake_callback(self, data):
    # position of snake head
    rospy.loginfo("Snake : ({}, {})".format(data.x, data.y))

  def score_callback(self, data):
    rospy.loginfo("Score : {}".format(data.data))

  def run(self):
    rospy.spin()

if __name__ == '__main__':
  try:
    server = SnakeClient()
    server.run()
  except rospy.ROSInterruptException:
    rospy.loginfo("Ros error")
    pass