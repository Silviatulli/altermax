import unittest
from src.minimax import Node, minimax
from src.GameModel import GameState


class TestMinimax(unittest.TestCase):
    def test_checkmate_in_one(self):
        # states in which the current player can win in 1 move
        winning_states = [
            18881,
            3917,
            26397,
            22601,
            3999,
            30089
        ]

        for state_id in winning_states:
            game_state = GameState.fromID(state_id)
            node = Node(is_child_turn=game_state.is_child_turn,
                        balls=game_state.balls)
            self.assertEqual(minimax(node, 3), 1)


def play_game(robot, child, isTraining=True):
    state = GameState()
    num_actions = 0

    while not state.isFinished():

        valid_actions = state.valid_actions()

        if state.is_child_turn:
            action, ball_id = child.policy(state)
            num_actions += 1

        else:
            action, ball_id = robot.policy(state)
            robot.give_explanation((action, ball_id), state)

        old_state = state
        state = state.make_action(action, ball_id)
        if isTraining:
            child.update(old_state, (action, ball_id), state)

    if state.is_child_turn and state.isFinished():
        outcome = -1
    elif not state.is_child_turn and state.isFinished():
        outcome = 1

    return outcome, num_actions


if __name__ == "__main__":
    robot = Robot()
    minimaxChild = ChildMinmax()

    num_episodes = 500
    win = 0
    for episode in tqdm(range(num_episodes)):
        outcome, num_actions = play_game(robot,
                                         child,
                                         isTraining=False)
        if outcome == 1:
            win += 1
    win_rate_child = win * 1.0/num_episodes
    print("MinimaxChild performance: {0}".format(win_rate_child))