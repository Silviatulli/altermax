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
  - returns the best actions, the POMDP, an explanation

- **child_decision.py**;
controls the child decision-making (q-learning)
  - represent q table, update child, perform an action
  
- **game_manager.py**;
links the view, the game model and the robot decision
  - plays the game
  - returns the child outcome and number of actions
  
- **robot_manager.py**;
controls the physical robot
  - speech, animations, facetracking
