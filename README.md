## Demo Simulator
run: python3 game_mananger.py

## File-by-File Details

- **game_model.py**;
the controls and the model of the game 
  - returns game states (balls position and score) and game actions
  - checks if the action is valid and if the game is finished
  - performs a valid action

- **minmax.py**;
standard implementation of minmax algorithm. The reference implementation is that one from [wikipedia](https://en.wikipedia.org/wiki/Minimax#Pseudocode)
  - returns a minimax tree (search-depth of 3), v(s), q-value
  
- **interface.py**;
view of the game
  - draw the board, the score, the balls, the outcome
  - update the game view

- **robot_decision.py**;
controls the robot decision-making (minmax search tree)
  - returns the best actions, the POMDP, a demonstration and an explanation (in the form of examples)

- **child_decision.py**;
controls the child decision-making

- **child_minmax.py**;
controls the child decision-making (minmax)

- **child_qlearning.py**;
controls the child decision-making (qlearning)
  - updates the q table when the child win or lose the game
  - updates the q table based on the robot state, action and reward (explanation update)
  - updates the q table based on a set of robot examples

- **child_evaluation.py**;
evaluates the child performance
  - train the child (qlearning)
  - define the win rate for minmax and qlearning
  - return the number of episodes that are necessary for the qlearning to perform as the minmax (baseline)

- **game_manager.py**;
links the view, the game model and the robot decision
  - plays the game
  - returns the child outcome and number of actions
