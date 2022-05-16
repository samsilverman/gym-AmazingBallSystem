"""
Module defining OpenAI gym environment for the Amazing Ball System
used in CS454/654: Embedded Systems Development.
"""
from pathlib import Path
import csv
from time import sleep
import math
import random
import gym
from gym import spaces
import numpy as np
import serial

SERIAL_X_DIM = 0
SERIAL_Y_DIM = 1

class AmazingBallSystemEnv(gym.Env):
    """
    Amazing Ball System gym environment

    ### Action Space

    The action is a `ndarray` with shape `(2,)` representing the percentage
    of servo control timing to apply to each motor.

    | Num | Action | Min  | Max |
    |-----|--------|------|-----|
    | 0   | X-motor % | 0 | 1.0 |
    | 1   | Y-motor % | 0 | 1.0 |

    Motors opperate in the range of 0 degrees (0.9ms duty) to 180 degrees (2.1 ms duty).
    Thus, percentages of 0 and 1 coorespond to duties of 0.9ms and 2.1ms respectively.

    For more information, see section 4.9 (Servos) of the lab manual.

    ### Observation Space

    The observation is a `ndarray` with shape `(2,)` representing the x-y coordinates of
    the ball constrained by the dimensions of the touchscreen board.

    | Num | Observation | Min  | Max |
    |-----|-------------|------|-----|
    | 0   | x   | min x | max x |
    | 1   | y   | min y | max y |

    For more information, see section 4.8 (Touchscreens) of the lab manual.

    ### Rewards

    The reward function is defined as:

    `r = -dist(pos, goal)`

    where `pos` is the ball's current position and goal is the goal position (center).

    `dist` is the distance from `pos` to `goal` normalized to the range [0, 1].

    ### Starting State

    The starting state is a random percentage for the x and y motors in the range [0, 1].

    ### Episode Termination

    The episode terminates at 100 time steps.
    """
    def __init__(self, port='/dev/ttyUSB0', calibrate=True, calibrate_file='calibration.csv'):
        """
        Initialize AmazingBallSystemEnv.

        Args:
            port:
                The usb port name connected the Amazing Ball System (default: `'/dev/ttyUSB0'`).
            calibrate:
                If `True`, calibrate the board and save results.
                If `False` load calibration (default: `True`).
            calibrate_file:
                The name of the calibration csv file to save/load board calibration from
                (default: `'calibration.csv'`).
        """
        super().__init__()

        self.serial = serial.Serial(port=port,
                                    baudrate=9600,
                                    bytesize=serial.EIGHTBITS,
                                    parity=serial.PARITY_NONE,
                                    stopbits=serial.STOPBITS_ONE,
                                    timeout=0.1)

        self.calibrate_file = Path(__file__).parent.resolve() / calibrate_file
        if calibrate is False and self.calibrate_file.is_file() is False:
            print(f'Calibration file {self.calibrate_file} does not exist. Running calibration...')
            calibrate = True

        if calibrate:
            self.x_range, self.y_range = self._calibrate()
        else:
            self.x_range, self.y_range = self._load_calibration()

        self.action_space = spaces.Box(low=0, high=1, shape=(2,), dtype=np.float16)

        self.observation_space = spaces.Box(low=np.array([self.x_range[0], self.y_range[0]]),
                                            high=np.array([self.x_range[1], self.y_range[1]]),
                                            dtype=np.float16)

        # center of board
        self.goal = [sum(self.x_range) // len(self.x_range), sum(self.y_range) // len(self.y_range)]

        # find furthest corner from center to be used in distance normalization
        corner_distances = [
            math.dist([self.x_range[0], self.y_range[0]], self.goal),
            math.dist([self.x_range[1], self.y_range[0]], self.goal),
            math.dist([self.x_range[1], self.y_range[1]], self.goal),
            math.dist([self.x_range[0], self.y_range[1]], self.goal),
        ]
        self.distance_max = max(corner_distances)

        self.current_step = 0
        self.max_step = 100
        self.done = False

    def step(self, action):
        """
        Update the environment according to an action.

        Args:
            action: The action to take.

        Returns:
            The next state, current reward, and done flag
        """
        self.current_step += 1
        if self.current_step >= self.max_step:
            self.done = True

        x_percent = action[0]
        y_percent = action[1]

        x_position = self._uart_comm(SERIAL_X_DIM, x_percent)
        y_position = self._uart_comm(SERIAL_Y_DIM, y_percent)

        # make sure ball position is in range of board
        x_position = max(min(self.x_range[1], x_position), self.x_range[0])
        y_position = max(min(self.y_range[1], y_position), self.y_range[0])

        state = np.array([x_position, y_position], dtype=np.uint16)

        reward = -math.dist([x_position, y_position], self.goal) / self.distance_max

        return state, reward, self.done, []

    def reset(self, seed=None):
        """
        Reset an episode of the environment.

        Returns:
            The initial state of the new episode.
        """
        super().reset(seed=seed)
        self.current_step = 0
        self.done = False
        self.position = [random.random(), random.random()]

        # move motors to new positon and return state
        self._uart_comm(SERIAL_X_DIM, self.position[0])
        self._uart_comm(SERIAL_Y_DIM, self.position[1])

        # wait for ball to move a little
        sleep(1)

        # send the same command to sample the ball's position
        x_position = self._uart_comm(SERIAL_X_DIM, self.position[0])
        y_position = self._uart_comm(SERIAL_Y_DIM, self.position[1])

        return np.array([x_position, y_position], dtype=np.uint16)

    def render(self):
        """
        TODO
        """
        pass

    def _load_calibration(self):
        """
        Load a board calibration csv file.

        Returns:
            The saved x range and y range of the board.
        """
        with open(self.calibrate_file, 'r', encoding='UTF8') as csv_file:
            reader = csv.DictReader(csv_file)
            for row in reader:
                x_min = int(row['x-min'])
                x_max = int(row['x-max'])
                y_min = int(row['y-min'])
                y_max = int(row['y-max'])
                return [x_min, x_max], [y_min, y_max]

    def _save_calibration(self):
        """
        Save a board calibration to csv file.
        """
        header = ['x-min', 'x-max', 'y-min', 'y-max']
        data = [self.x_range[0], self.x_range[1], self.y_range[0], self.y_range[1]]
        with open(self.calibrate_file, 'w', encoding='UTF8', newline='') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(header)
            writer.writerow(data)

    def _calibrate(self):
        """
        Calibrate the Amazing Ball System board ranges.

        The more conservative values for the x and y range will be used.

        For example, if the min x is found to be 100 and 110 at two locations,
        the value of 110 is used.
        """
        x_min, y_min = self._calibrate_corner(x_perc=0, y_perc=0)

        x_max, y_temp = self._calibrate_corner(x_perc=1, y_perc=0)
        if y_temp > y_min:
            y_min = y_temp

        x_temp, y_max = self._calibrate_corner(x_perc=1, y_perc=1)
        if x_temp < x_max:
            x_max = x_temp

        x_temp, y_temp = self._calibrate_corner(x_perc=0, y_perc=1)
        if x_temp > x_min:
            x_min = x_temp
        if y_temp < y_max:
            y_max = y_temp

        return [x_min, x_max], [y_min, y_max]

    def _calibrate_corner(self, x_perc, y_perc):
        """
        Move ball to a desired corner of the Amazing Ball System for calibration.

        Args:
            x_perc: The percentage for the x motor.
            y_perc: The percentage for the y motor.

        Returns:
            The ball's position in the desired corner.
        """
        self._uart_comm(SERIAL_X_DIM, x_perc)
        self._uart_comm(SERIAL_Y_DIM, y_perc)

        # wait for ball to move to the corner
        sleep(3)

        # send the same command to sample the ball's position
        x_position = self._uart_comm(SERIAL_X_DIM, x_perc)
        y_position = self._uart_comm(SERIAL_Y_DIM, y_perc)

        return x_position, y_position

    def _uart_comm(self, dimension, percentage):
        """
        Send command to Amazing Ball System Motor over UART.

        Args:
            dimension: The motor dimension (SERIAL_X_DIM, or SERIAL_X_DIM).
            percentage: The motor duty cycle [0.9, 2.1] as a percentage [0, 1].

        Returns:
            The position of the ball in the `dimension` dimension.
        """
        # give touchscreen time to switch sample direction
        sleep(0.01)

        duty = (int)(900 + (percentage * 1200))
        duty_low_bits, duty_high_bits = (duty & 0xFFFFFFFF).to_bytes(2, 'big')
        message = [0, dimension, duty_low_bits, duty_high_bits]
        message_bytes = bytearray(message)
        self.serial.write(message_bytes)

        response_low_bytes = self.serial.read()
        response_high_bytes = self.serial.read()
        response_bytes = response_low_bytes + response_high_bytes
        response = int.from_bytes(response_bytes, 'big')

        return np.uint16(response)
