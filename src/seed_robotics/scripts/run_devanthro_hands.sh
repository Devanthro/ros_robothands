#!/bin/bash

SESSION="hands"
tmux new-session -d -s $SESSION

# Split panes
tmux split-window -v
tmux split-window -h
tmux select-pane -t 0
tmux split-window -h

# Run commands in each pane
tmux select-pane -t 0
tmux send-keys "rosrun seed_robotics map_hand_commands.py" C-m

tmux select-pane -t 1
tmux send-keys "rosrun seed_robotics filter_js.py" C-m

tmux select-pane -t 2
tmux send-keys "roslaunch seed_robotics RH8D_R_devanthro.launch" C-m

tmux select-pane -t 3
tmux send-keys "roslaunch seed_robotics RH8D_L_devanthro.launch" C-m

# Attach to session
tmux attach-session -t $SESSION