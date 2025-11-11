#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import pygame

key_map = {
    pygame.K_w: ('x', 1.0),
    pygame.K_s: ('x', -1.0),
    pygame.K_a: ('y', 1.0),
    pygame.K_d: ('y', -1.0),
    pygame.K_LEFT: ('th', 1.0),
    pygame.K_RIGHT: ('th', -1.0),
    pygame.K_DOWN: ('z', -1.0),
    pygame.K_UP: ('z', 1.0)
}

usage_lines = [
    "Keyboard Control for Robot (cmd_vel)",
    "--------------------------------------",
    "  w: move forward",
    "  s: move backward",
    "  a: move left",
    "  d: move right",
    "  LEFT: turn left",
    "  RIGHT: turn right",
    "  Supports multiple keys pressed at once",
    "  Press: set speed to 1m/s; Release: clear only that axis",
    "  q: quit",
    "--------------------------------------",
    "Make sure this window is focused!"
]

def draw_usage(screen, font):
    screen.fill((30, 30, 30))
    for i, line in enumerate(usage_lines):
        text = font.render(line, True, (220, 220, 220))
        screen.blit(text, (10, 10 + i * 28))

def main():
    rospy.init_node('pygame_teleop')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    pygame.init()
    win_width, win_height = 420, 28 * len(usage_lines) + 20
    screen = pygame.display.set_mode((win_width, win_height))
    pygame.display.set_caption("ROS Teleop Instructions")

    font = pygame.font.SysFont(None, 24)
    control = {'x': 0.0, 'y': 0.0, 'z': 0, 'th': 0.0}
    pressed = set()
    running = True
    clock = pygame.time.Clock()

    while running and not rospy.is_shutdown():
        draw_usage(screen, font)
        pygame.display.flip()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q:
                    running = False
                elif event.key in key_map and event.key not in pressed:
                    axis, value = key_map[event.key]
                    control[axis] += value
                    pressed.add(event.key)
                    twist = Twist()
                    twist.linear.x = control['x']
                    twist.linear.y = control['y']
                    twist.angular.z = control['th']
                    twist.linear.z = control['z']
                    pub.publish(twist)
            elif event.type == pygame.KEYUP:
                if event.key in key_map and event.key in pressed:
                    axis, value = key_map[event.key]
                    control[axis] -= value
                    pressed.remove(event.key)
                    twist = Twist()
                    twist.linear.x = control['x']
                    twist.linear.y = control['y']
                    twist.linear.z = control['z']
                    twist.angular.z = control['th']
                    
                    pub.publish(twist)
        clock.tick(50)

    pub.publish(Twist())
    pygame.quit()
    print("\nExit keyboard teleop.")

if __name__ == '__main__':
    main()
