#!/bin/bash

# Define the tmux session name
SESSION="gs_session"

# Kill the program in the left panel (Pane 0)
tmux send-keys -t $SESSION:0.0 C-c  # Send Ctrl+C to stop the script
tmux send-keys -t $SESSION:0.0 "exit" C-m  # Exit the shell in the pane

# Kill the program in the top-right panel (Pane 1)
tmux send-keys -t $SESSION:0.1 C-c  # Send Ctrl+C to stop the script
tmux send-keys -t $SESSION:0.1 "exit" C-m  # Exit the shell in the pane

# Kill the program in the bottom-right panel (Pane 2)
tmux send-keys -t $SESSION:0.2 C-c  # Send Ctrl+C to stop the script
tmux send-keys -t $SESSION:0.2 "exit" C-m  # Exit the shell in the pane

# Optionally, kill the tmux session completely
tmux kill-session -t $SESSION

echo "All programs and tmux session terminated."