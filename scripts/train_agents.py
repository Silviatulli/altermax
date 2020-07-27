import tensorflow as tf
import json
from sklearn.model_selection import KFold
import jsonpickle
import numpy as np

from minmax.game_model import GameState


def load_data(file_name):
    with open(file_name, "r") as file:
        data = json.load(file)
        data_decoded = jsonpickle.decode(data)

    states = list()
    actions = list()
    rewards = list()
    next_states = list()

    for state, action, reward, next_state in data_decoded:
        state_idx = GameState.get_state_id(state)
        states.append(state_idx)

        next_state_idx = GameState.get_state_id(next_state)
        next_states.append(next_state_idx)

        actions.append(int(action))

        rewards.append(reward)

    states = np.stack(states)
    actions = np.stack(actions)
    rewards = np.stack(rewards)
    next_states = np.stack(next_states)

    return states, actions, rewards, next_states


def q_learning_model():
    NUM_STATES = 12*12*12*12*2
    NUM_ACTIONS = 18
    GAMMA = 0.99

    model_in = tf.keras.layers.Input(shape=(1,), dtype=tf.int32)
    tmp = tf.one_hot(model_in, NUM_STATES)
    tmp = tf.keras.layers.Dense(NUM_ACTIONS, use_bias=False)(tmp)
    model_out = tf.squeeze(tmp, axis=1)
    q_function = tf.keras.Model(model_in, model_out)

    state = tf.keras.layers.Input(shape=(1,), dtype=tf.int32, name="State")
    action = tf.keras.layers.Input(shape=(1,), dtype=tf.int32, name="Action")
    reward = tf.keras.layers.Input(shape=(1,), name="Reward")
    next_state = tf.keras.layers.Input(
        shape=(1,), dtype=tf.int32, name="Next_State")

    td_target = reward + GAMMA * tf.reduce_max(q_function(next_state), axis=-1)
    predictions = tf.gather(q_function(state), action, axis=-1)
    train_model = tf.keras.Model(
        inputs=[state, action, reward, next_state],
        outputs=[predictions, td_target]
    )

    # to date it still feels as if tf.stop_gradient is a horrible
    # hack similar to DDQL to stabelize the algorithm
    td_error = 0.5 * tf.abs(tf.stop_gradient(td_target) - predictions) ** 2
    train_model.add_loss(td_error, [state, action, reward, next_state])

    predicted_action = tf.argmax(q_function(state), axis=-1)
    correct_predictions = tf.keras.metrics.categorical_accuracy(
        action, predicted_action)
    train_model.add_metric(correct_predictions,
                           name="Matched_Actions", aggregation="mean")

    return q_function, train_model


def policy_gradient():
    NUM_STATES = 12*12*12*12*2
    NUM_ACTIONS = 18
    GAMMA = 0.99

    model_in = tf.keras.layers.Input(shape=(1,), dtype=tf.int32)
    tmp = model_in
    tmp = tf.keras.layers.Dense(NUM_ACTIONS, activation="softmax")(tmp)
    model_out = 1.0 * tmp
    policy_fn = tf.keras.Model(model_in, model_out)

    state = tf.keras.layers.Input(shape=(1,), dtype=tf.int32, name="State")
    action = tf.keras.layers.Input(shape=(1,), dtype=tf.int32, name="Action")
    reward = tf.keras.layers.Input(shape=(1,), name="Reward")
    next_state = tf.keras.layers.Input(
        shape=(1,), dtype=tf.int32, name="Next_State")

    target = tf.one_hot(action, NUM_ACTIONS)
    target = tf.squeeze(target, axis=1)
    predictions = policy_fn(state)
    train_model = tf.keras.Model(
        inputs=[state, action], outputs=[predictions, target])

    error = tf.keras.losses.categorical_crossentropy(target, predictions)
    train_model.add_loss(error, [state, action])

    most_likely_action = tf.argmax(policy_fn(state), axis=-1)
    correct_predictions = tf.keras.metrics.categorical_accuracy(
        action, most_likely_action)
    train_model.add_metric(correct_predictions,
                           name="Matched_Actions", aggregation="mean")

    return policy_fn, train_model


if __name__ == "__main__":
    states, actions, rewards, next_states = load_data("data.json")
    indices = np.arange(len(states))

    # # Q Learning
    # # ===========
    # q_function, train_q = q_learning_model()

    # # training the q-learning agent
    # train_q.compile(optimizer="sgd")
    # train_q.fit([states, actions, rewards, next_states])

    # # using the learned model
    # q_values = q_function(states).numpy()
    # best_actions = np.argmax(q_values, axis=-1)

    # # Policy Gradient
    # # ================
    # policy_fn, train_policy = policy_gradient()

    # # training the policy gradient
    # train_policy.compile(optimizer="sgd")
    # train_policy.fit([states, actions, rewards, next_states])

    # # use the learned model
    # action_props = policy_fn(states).numpy()
    # cum_prop = np.cumsum(action_props, axis=-1)
    # rng = np.random.rand(len(action_props))[..., np.newaxis]
    # best_actions = np.argmax(rng <= cum_prop, axis=-1)

    q_scores = list()
    policy_scores = list()
    for train_idx, test_idx in KFold(shuffle=True).split(indices):
        train_data = [
            states[train_idx, ...],
            actions[train_idx, ...],
            rewards[train_idx, ...],
            next_states[train_idx, ...],
        ]
        test_data = [
            states[test_idx, ...],
            actions[test_idx, ...],
            rewards[test_idx, ...],
            next_states[test_idx, ...],
        ]

        q_function, train_q = q_learning_model()
        train_q.compile(optimizer="sgd")
        train_q.fit(train_data)
        _, score = train_q.evaluate(test_data)
        q_scores.append(score)

        policy_fn, train_policy = q_learning_model()
        train_policy.compile(optimizer="sgd")
        train_policy.fit(train_data)
        _, score = train_policy.evaluate(test_data)
        policy_scores.append(score)

q_scores = np.array(q_scores)
print(f"Q-Learning Accuracy: M={np.mean(q_scores):.2f} "
      f"(SD={np.std(q_scores):.2f})")

policy_scores = np.array(policy_scores)
print(f"Policy-Iteration Accuracy: M={np.mean(policy_scores):.2f} "
      f"(SD={np.std(policy_scores):.2f})")
