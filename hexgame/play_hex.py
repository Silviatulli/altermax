import pygame
import configparser
import numpy as np
from math import sin, cos, pi
from itertools import product
import minihex
from minihex import player
import gym
import random


def draw_stone(surface, pos, player, config):
    radius = 0.6 * float(config["GameWindow"]["hexagon_radius"])

    if player == player.BLACK:
        color = pygame.Color(config["GameWindow"]["player1_stone_color"])
    else:
        color = pygame.Color(config["GameWindow"]["player2_stone_color"])

    pos = (int(np.round(pos[0])), int(np.round(pos[1])))
    pygame.draw.circle(surface, color, pos, int(np.round(radius)))
    border_color = pygame.Color(config["GameWindow"]["stone_border_color"])
    pygame.draw.circle(surface, border_color, pos, int(np.round(radius)), 1)


def hexagon_points(radius, offset=(0, 0)):
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

    return points_pos + points_neg


def test_line(pointA, pointB, test_point):
    x1, y1 = pointA
    x2, y2 = pointB
    x, y = test_point

    return (x - x1) * (y2 - y1) - (y - y1) * (x2 - x1)


def inside_hexagon(midpoint, radius, test_point):
    mx, my = midpoint
    x, y = test_point

    # outside envelopping circle
    if np.sqrt((x - mx)**2 + (y - my)**2) > radius:
        return False

    # inside containing circle
    if np.sqrt((x - mx)**2 + (y - my)**2) < radius * np.sqrt(3) / 2:
        return True

    # between enveloping and containing circles
    # perform generic polygon test
    points = hexagon_points(radius, midpoint)
    sign = np.sign(test_line(points[0], points[1], test_point))
    for idx in range(1, len(points) - 1):
        pointA = points[idx]
        pointB = points[idx + 1]
        val = np.sign(test_line(pointA, pointB, test_point))
        if sign != val:
            return False

    return True


def pos2idx(test_point, config):
    positions = board_positions(config)
    radius = int(config["GameWindow"]["hexagon_radius"])

    for idx, pos in enumerate(positions):
        if inside_hexagon(pos, radius, test_point):
            return idx

    return None


def board_positions(config):
    game_conf = config["GameWindow"]
    long_radius = int(game_conf["hexagon_radius"])
    short_radius = np.sqrt(3) / 2 * long_radius
    board_size = int(config["HexGame"]["board_size"])

    col_px = [idx * 1.5 * long_radius + long_radius
              for idx in range(2 * board_size - 1)]
    row_px = [(idx + 1) * short_radius
              for idx in range(2 * board_size - 1)]

    stones_per_row = ([idx for idx in range(1, board_size)] +
                      [idx for idx in range(board_size, 0, -1)])
    start_idx = ([idx for idx in range(board_size - 1, 0, -1)] +
                 [idx for idx in range(board_size)])

    center_positions = list()
    for offset in range(board_size):
        y_start = board_size - 1 + offset
        x_start = offset
        for row_counter in range(board_size):
            y_idx = y_start - row_counter
            x_idx = x_start + row_counter
            center_positions.append((col_px[x_idx], row_px[y_idx]))

    return center_positions


def draw_empty_board(surface, hover_idx, config):
    game_conf = config["GameWindow"]
    surface.fill(pygame.Color(game_conf["background_color"]))

    long_radius = int(game_conf["hexagon_radius"])
    short_radius = np.sqrt(3) / 2 * long_radius
    board_size = int(config["HexGame"]["board_size"])

    positions = board_positions(config)

    for idx, pos in enumerate(positions):
        edges = hexagon_points(long_radius, pos)
        if hover_idx == idx:
            pygame.draw.polygon(surface, pygame.Color(
                game_conf["hexagon_accent_color"]), edges)
        else:
            pygame.draw.polygon(surface, pygame.Color(
                game_conf["hexagon_color"]), edges)
        pygame.draw.polygon(surface, pygame.Color(
            game_conf["hexagon_border_color"]), edges, 3)


def draw_env(surface, sim, hover_idx, config):
    game_conf = config["GameWindow"]
    surface.fill(pygame.Color(game_conf["background_color"]))

    long_radius = int(game_conf["hexagon_radius"])
    short_radius = np.sqrt(3) / 2 * long_radius
    board_size = int(config["HexGame"]["board_size"])

    positions = board_positions(config)
    occupancy = sim.board.ravel()

    left = (positions[0][0] - 2.5 * long_radius, positions[0][1])
    right = (positions[-1][0] + 2.5 * long_radius, positions[-1][1])
    top = (positions[board_size - 1][0],
           positions[board_size - 1][1] - 2 * long_radius)
    bottom = (positions[board_size - 1][0],
              positions[(board_size - 1) * board_size][1] + 2 * long_radius)
    center = positions[len(positions) // 2]

    color = pygame.Color(config["GameWindow"]["player1_stone_color"])
    pygame.draw.polygon(surface, color, [left, top, center])
    pygame.draw.polygon(surface, color, [center, bottom, right])

    color = pygame.Color(config["GameWindow"]["player2_stone_color"])
    pygame.draw.polygon(surface, color, [top, right, center])
    pygame.draw.polygon(surface, color, [bottom, left, center])

    for idx, pos in enumerate(positions):
        edges = hexagon_points(long_radius, pos)
        if hover_idx == idx:
            pygame.draw.polygon(surface, pygame.Color(
                game_conf["hexagon_accent_color"]), edges)
        else:
            pygame.draw.polygon(surface, pygame.Color(
                game_conf["hexagon_color"]), edges)
        pygame.draw.polygon(surface, pygame.Color(
            game_conf["hexagon_border_color"]), edges, 3)

        if occupancy[idx] == player.BLACK:
            draw_stone(surface, pos, player.BLACK, config)
        elif occupancy[idx] == player.WHITE:
            draw_stone(surface, pos, player.WHITE, config)


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

    done = False
    running = True
    while running:
        mouse_pos = pygame.mouse.get_pos()
        hover_idx = pos2idx(mouse_pos, config)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            if event.type == pygame.MOUSEBUTTONUP:
                if hover_idx is None:
                    continue

                action = hover_idx
                try:
                    state, reward, done, info = env.step(action)
                except:
                    # illegal move; ignore
                    continue

        if done:
            state, info = env.reset()

        draw_env(surface, env.simulator, hover_idx, config)
        pygame.display.update()


if __name__ == "__main__":
    config = configparser.ConfigParser()
    config.read('hexgame/hexconfig.ini')
    main(config)
