#!/usr/bin/env python

import pygame
import rospy

from std_msgs.msg import String, Int32
from snake_game.msg import position

from snake_world import SnakeWorld

class SnakeServer:
  def __init__(self):

    # Initializing the ros node
    rospy.init_node('snake_server', anonymous=True)

    # Initializing the Snake World
    self.world = SnakeWorld()
    rospy.loginfo("Snake World Loaded")

    # Initializing the Publisher
    self.pub_score = rospy.Publisher('/snake_game/score', Int32, queue_size=1)
    self.pub_food = rospy.Publisher('/snake_game/food', position, queue_size=10)
    self.pub_snake = rospy.Publisher('/snake_game/snake', position, queue_size=10)

    # Initializing the Subscriber
    rospy.Subscriber("/snake_game/move", String, self.move_snake_callback)

    # Initializing the Rate
    self.rate = rospy.Rate(10) # 10hz

    self.episodes_cnt = 0
    self.episode = 0
    self.episode_running = True

  def move_snake_callback(self, data):
    if data.data != 'PAUSE':
      # rospy.loginfo("Move Snake to %s", data.data)
      try:
        self.world.move_snake(data.data)
      except Exception:
        rospy.loginfo("You Lose")
        self.episode_running = False

  def run_episode(self):
    rospy.loginfo("Starting New Episode {}".format(self.episode + 1))

    self.episode_running = True
    while self.episode_running:
      # fetching pressed key
      for event in pygame.event.get(): 
        if event.type == pygame.QUIT:
          self.episode_running = False
          self.episode = self.episodes_cnt

      food_pos = position()
      food_pos.x, food_pos.y = self.world.food

      snake_pos = position()
      snake_pos.x, snake_pos.y = self.world.snake

      # publish -> score, food_pos, snake_head_pos
      self.pub_score.publish(self.world.score)
      self.pub_food.publish(food_pos)
      self.pub_snake.publish(snake_pos)
      
      # Refresing the Game screen
      pygame.display.flip()

    rospy.loginfo("Episode {} Finish".format(self.episode + 1))

  def run(self, episodes_cnt):
    self.episodes_cnt = episodes_cnt
    while self.episode < self.episodes_cnt:
      self.world.restart()
      self.run_episode()
      self.episode += 1

    rospy.loginfo("Shutdown Sever")

if __name__ == '__main__':
  try:
    episodes_cnt = 5
    server = SnakeServer()
    server.run(episodes_cnt)
  except rospy.ROSInterruptException:
    rospy.loginfo("Ros error")
    pass