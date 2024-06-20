__credits__ = ["Pablo ALGAR"]

import math
from typing import Optional, Union

import numpy as np

import gymnasium as gym
from gymnasium import spaces
from gymnasium.error import DependencyNotInstalled, InvalidAction


try:
    # As pygame is necessary for using the environment (reset and step) even without a render mode
    #   therefore, pygame is a necessary import for the environment.
    import pygame
except ImportError as e:
    raise DependencyNotInstalled(
        "pygame is not installed, run `pip install gymnasium[box2d]`"
    ) from e


PLAYFIELD = 5000 # milimeters
MARGIN = 500 # milimeters

ZOOM = 10
FPS = 30  # Frames per second
MAX_TRACK_SPEED = 1000.0  # milimeters per second
MIN_TRACK_SPEED = 0.5  # milimeters per second

CAR_SPEED = 1000.0  # milimeters per second
CAR_TURNING_RADIUS = 1000.0  # milimeters

class TrackerCar(gym.Env):
    """
    ## Description
    ## Description
    
## Description    
    
Lateral position and distance of the ball  are indicated

## Action space

Discrete:
 - steer left
 - steer center
 - steer right
 - forward
 - stop
 - backward

## Observation space

- size of the ball in pixels
- horizontal position of the ball

##Rewards

 Based on how close the ditance is to the target distance and how centered the ball is

## Starting state

The car starts at the center of a box with the ball in front of it at the target distance
A spline is generated for the ball to move.
The spline is defined by 7 control vectors, randomly positioned and with random speed between 0.5 and 1.0 m/s, the first and last of them being 0m/s.

## Episode termination

The episode terminates qhen the splines finishes or the car exists the box    

## Arguments

## Reset Arguments

## Version History
- v0: Original version
"""

    metadata = {
        "render_modes": [
            "human"        ],
        "render_fps": FPS,
    }

    def bezier(t, P0, P1, P2, P3):
        return (1-t)**3 * P0 + 3 * (1-t)**2 * t * P1 + 3 * (1-t) * t**2 * P2 + t**3 * P3


    def get_track(np_random):
        x_pos = np_random.uniform( MARGIN, PLAYFIELD - MARGIN)
        y_pos = np_random.uniform( MARGIN, PLAYFIELD - MARGIN)
        speed = np_random.uniform(MIN_TRACK_SPEED, MAX_TRACK_SPEED)
        bearing = np_random.uniform(0, 2 * math.pi)
        x_speed = speed * math.cos(bearing)
        y_speed = speed * math.sin(bearing)

        return x_pos, y_pos, x_speed, y_speed


    def __init__(self, render_mode=None, size=5):

        self.screen: Optional[pygame.Surface] = None
        self.clock = None
        self.spline = None
        self.reward = 0.0
        self.prev_reward = 0.0

        # left, center, right, forward, stop, backward
        self.action_space = spaces.Discrete(6)
        self.observation_space = spaces.Box(0, size - 1, shape=(2,), dtype=np.float32)

        self.render_mode = render_mode

        CONTROLPOINTS = 7

        # Create trajectory
        x1_pos, y1_pos, x1_speed, y1_speed = PLAYFIELD / 2, PLAYFIELD / 2, 0.0, 0.0
        self.coordinates = []
        for c in range(CONTROLPOINTS):
            x2_pos, y2_pos, x2_speed, y2_speed = self.get_track(self.np_random)

            P0 = np.array([x1_pos, y1_pos])
            P3 = np.array([x2_pos, y2_pos])
            V0 = np.array([x1_speed, x2_speed])
            V3 = np.array([x2_speed, y2_speed])

            P1 = P0 + V0
            P2 = P3 + V3

            # track duration according to Euclidean distance and car speed
            distance = np.linalg.norm(P3 - P0) * 2.0
            T = distance / CAR_SPEED
            dt = 1.0 / FPS

            for t in np.arange(0, 1 + dt/(T * dt), dt/(T * dt)):
                point = self.bezier(t, P0, P1, P2, P3)
                self.coordinates.append(point)

        self.total_coordinates = len(self.coordinates)
        self.current_coordinate = 0

    def reset(
        self,
        *,
        seed: Optional[int] = None,
        options: Optional[dict] = None,
    ):
        super().reset(seed=seed)
        self._destroy()
        self.reward = 0.0
        self.prev_reward = 0.0
        self.tile_visited_count = 0
        self.t = 0.0

        if self.render_mode == "human":
            self.render()
        return self.step(None)[0], {}

    def step(self, action: Union[np.ndarray, int]):
            
        if not self.action_space.contains(action):
            raise InvalidAction(
                f"you passed the invalid action `{action}`. "
                f"The supported action_space is `{self.action_space}`"
            )


        self.world.Step(1.0 / FPS, 6 * 30, 2 * 30)
        self.t += 1.0 / FPS


        step_reward = 0
        terminated = False
        truncated = False
        if action is not None:  # First step without action, called from reset()
            if abs(x) > PLAYFIELD or abs(y) > PLAYFIELD:
                terminated = True
                step_reward = -100

        if self.render_mode == "human":
            self.render()
        return self.state, step_reward, terminated, truncated, {}

    def render(self):
        if self.render_mode == "human":
            return self._render_frame()

    def _render_frame(self):
        if self.window is None and self.render_mode == "human":
            pygame.init()
            pygame.display.init()
            self.window = pygame.display.set_mode(
                (self.window_size, self.window_size)
            )
        if self.clock is None and self.render_mode == "human":
            self.clock = pygame.time.Clock()

        canvas = pygame.Surface((PLAYFIELD//ZOOM, PLAYFIELD//ZOOM))
        canvas.fill((0, 0, 0))

        # draw the track ball
        x = int(self.coordinates[self.current_coordinate][0]//ZOOM)
        y = int(self.coordinates[self.current_coordinate][1]//ZOOM)
        pygame.draw.circle(canvas, (255, 255, 0), (x, y, 10))

        # Update the display
        pygame.display.flip()
        self.clock.tick(FPS)

    def close(self):
        if self.screen is not None:
            pygame.display.quit()
            self.isopen = False
            pygame.quit()

