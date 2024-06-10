# ROS topic page
This page allows user to check rostopics available.

For ROS1 user, make sure `ROS Master` is exported to the correct IP addresss, and machines have listed in the `/etc/hosts` file in all machines

- `Get ROS Topics`: the backend software will execute `rostopic list` command **on the machine that is running mvp_gui (e.g., a topside computer)**.

- `Keywords`: user could type comma separated keywords in the textbox then click `Get ROS Topics` to display filtered rotpic list. The keywords can be removed in the ROS Topic Keywords table.

- `ROS Topic Lists` table will list all the (filtered) ros topics. User could click `Echo Once` to get a recent message. If the wind showing `empty`, user needs to check `ROS Master` is correctly exported and `/etc/hosts` list on both machines (gui machine and the vehicle machine).



