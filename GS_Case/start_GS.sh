#!/bin/bash

# Define paths to your shell scripts
SCRIPT1="/home/pi/avionics/cansat/SW/GSW/GS_Case/run_gs_app_c.sh"
SCRIPT2="/home/pi/avionics/cansat/SW/GSW/GS_Case/run_gs_app_python.sh"
SCRIPT3="/home/pi/avionics/cansat/SW/GSW/dashboard/SherpaDashboardKufr/startServer.py"
SESSION="gs_session"

# Open a new terminal window and run tmux session inside it
lxterminal -e "
    # Start a new tmux session detached
    tmux new-session -d -s $SESSION

    # Run the first script in the left pane
    tmux send-keys -t $SESSION 'bash $SCRIPT1' C-m

    # Split the window horizontally for the second script
    tmux split-window -h -t $SESSION
    tmux send-keys -t $SESSION:0.1 'bash $SCRIPT2' C-m

    # Split the right pane vertically for the third script
    tmux split-window -v -t $SESSION:0.1
    tmux send-keys -t $SESSION:0.2 'cd $(dirname $SCRIPT3) && python3 $(basename $SCRIPT3)' C-m

    # Select the first pane (left side) to start
    tmux select-pane -t $SESSION:0.0

    # Attach to the tmux session
    tmux attach -t $SESSION
"