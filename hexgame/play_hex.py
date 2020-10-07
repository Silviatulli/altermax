import pygame
import configparser
import numpy as np
from math import sin, cos, pi
from itertools import product
import minihex
from minihex import player
import gym
import random


class Hexagon(object):
    def __init__(self, config, offset=(0, 0)):
        self.hex_config = config["Hexagon"]
        self.stone_config = config["Stone"]
        radius = float(self.hex_config["radius"])

        offset = np.asarray(offset).reshape((-1, 1))
        rot_matrix = np.array([
            [cos(pi / 3), -sin(pi / 3)],
            [sin(pi / 3), cos(pi / 3)]
        ])
        vector = np.array([[radius], [0]])

        points_pos = list()
        points_neg = list()
        for _ in range(3):
            points_pos.append((vector + offset).ravel().tolist())
            points_neg.append((offset - vector).ravel().tolist())
            vector = np.matmul(rot_matrix, vector)

        self.points = points_pos + points_neg
        self.midpoint = offset

    def draw(self, surface, mouse_pos, stone=player.EMPTY):
        if self.contains(mouse_pos):
            color = pygame.Color(self.hex_config["primary_color"])
        else:
            color = pygame.Color(self.hex_config["color"])
        pygame.draw.polygon(surface, color, self.points)

        border_color = pygame.Color(self.hex_config["border_color"])
        pygame.draw.polygon(surface, border_color, self.points, 3)

        if stone == player.EMPTY:
            return
        elif stone == player.WHITE:
            color = pygame.Color(self.stone_config["player2_color"])
        elif stone == player.BLACK:
            color = pygame.Color(self.stone_config["player1_color"])

        radius = int(self.stone_config["radius"])
        x = int(np.round(self.midpoint[0]))
        y = int(np.round(self.midpoint[1]))
        pygame.draw.circle(surface, color, (x, y), int(np.round(radius)))

        border_color = pygame.Color(self.stone_config["border_color"])
        pygame.draw.circle(surface, border_color, (x, y), radius, 1)

    def contains(self, point):
        radius = int(self.hex_config["radius"])
        mx, my = self.midpoint
        x, y = point

        # outside envelopping circle
        if np.sqrt((x - mx)**2 + (y - my)**2) > radius:
            return False

        # inside containing circle
        if np.sqrt((x - mx)**2 + (y - my)**2) < radius * np.sqrt(3) / 2:
            return True

        # between enveloping and containing circles
        # perform generic polygon test
        points = self.points
        sign = np.sign(self._test_line(points[0], points[1], point))
        for idx in range(1, len(points) - 1):
            pointA = points[idx]
            pointB = points[idx + 1]
            val = np.sign(self._test_line(pointA, pointB, point))
            if sign != val:
                return False

        return True

    def _test_line(self, pointA, pointB, test_point):
        x1, y1 = pointA
        x2, y2 = pointB
        x, y = test_point

        return (x - x1) * (y2 - y1) - (y - y1) * (x2 - x1)


class Board(object):
    def __init__(self, pos, config):
        self.centers = self._board_positions(config, pos)
        self.hexagons = list()
        self.config = config
        for pos in self.centers:
            self.hexagons.append(Hexagon(config, offset=pos))

    def draw(self, surface, sim, mouse_pos):
        occupancy = sim.board.ravel()

        bg_color = pygame.Color(self.config["GameWindow"]["background_color"])
        surface.fill(bg_color)

        self._draw_game_border(surface)

        for stone, hexagon in zip(occupancy, self.hexagons):
            if hexagon.contains(mouse_pos):
                hexagon.hovered = True
            else:
                hexagon.hovered = False

            hexagon.draw(surface, mouse_pos, stone)

    def _draw_game_border(self, surface):
        long_radius = int(self.config["Hexagon"]["radius"])
        short_radius = np.sqrt(3) / 2 * long_radius
        board_size = int(config["HexGame"]["board_size"])

        positions = self.centers

        x_size = (board_size + board_size // 2) * long_radius
        y_size = board_size * short_radius
        x_offset = 1.5 * long_radius
        y_offset = 1 * long_radius

        center = positions[len(positions) // 2]
        left = (center[0] - x_size - x_offset, center[1])
        right = (center[0] + x_size + x_offset, center[1])
        top = (center[0], center[1] - y_size - y_offset)
        bottom = (center[0], center[1] + y_size + y_offset)

        color = pygame.Color(config["Stone"]["player1_color"])
        pygame.draw.polygon(surface, color, [left, top, center])
        pygame.draw.polygon(surface, color, [center, bottom, right])

        color = pygame.Color(config["Stone"]["player2_color"])
        pygame.draw.polygon(surface, color, [top, right, center])
        pygame.draw.polygon(surface, color, [bottom, left, center])

    def pos2idx(self, test_point):
        for idx, hexagon in enumerate(self.hexagons):
            if hexagon.contains(test_point):
                return idx
        return None

    def _board_positions(self, config, offset):
        long_radius = int(config["Hexagon"]["radius"])
        short_radius = np.sqrt(3) / 2 * long_radius
        board_size = int(config["HexGame"]["board_size"])

        col_px = [idx * 1.5 * long_radius + long_radius + offset[0]
                  for idx in range(2 * board_size - 1)]
        row_px = [(idx + 1) * short_radius + offset[1]
                  for idx in range(2 * board_size - 1)]

        stones_per_row = ([idx for idx in range(1, board_size)] +
                          [idx for idx in range(board_size, 0, -1)])
        start_idx = ([idx for idx in range(board_size - 1, 0, -1)] +
                     [idx for idx in range(board_size)])

        center_positions = list()
        for start_idx in range(board_size):
            y_start = board_size - 1 + start_idx
            x_start = start_idx
            for row_counter in range(board_size):
                y_idx = y_start - row_counter
                x_idx = x_start + row_counter
                center_positions.append((col_px[x_idx], row_px[y_idx]))

        return center_positions


def main(config):
    game_conf = config["GameWindow"]
    board_size = int(config["HexGame"]["board_size"])

    pygame.init()
    # load and set the logo
    # logo = pygame.image.load("logo32x32.png")
    # pygame.display.set_icon(logo)
    pygame.display.set_caption(game_conf["window_name"])

    screen_x, screen_y = game_conf["screen_size"].split(sep="x")
    surface = pygame.display.set_mode((int(screen_x), int(screen_y)))

    env = gym.make("hex-v0",
                   opponent_policy=minihex.random_policy,
                   board_size=board_size)
    state, info = env.reset()

    board = Board((50, 50), config)

    done = False
    running = True
    while running:
        mouse_pos = pygame.mouse.get_pos()
        hover_idx = board.pos2idx(mouse_pos)
        board.draw(surface, env.simulator, mouse_pos)
        pygame.display.update()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            if event.type == pygame.MOUSEBUTTONUP:
                if hover_idx is None:
                    continue

                action = hover_idx
                try:
                    state, reward, done, info = env.step(action)
                except IndexError:
                    # ignore illegal move
                    continue

        if done:
            state, info = env.reset()


if __name__ == "__main__":
    config = configparser.ConfigParser()
    config.read('hexgame/hexconfig.ini')
    main(config)
