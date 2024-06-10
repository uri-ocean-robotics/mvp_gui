# Map page
This is the main page for vehicle pilots

- `Controller state` table allows user to `enabled` or `disable` controller on the vehicle with current status show on the left.

- `State` table shows the vehicle's current state, and connected states. User can click the connected state to call the state change service.

The service names for the controller and the state machine are listed in the `config/gui_config.yaml` file.

```
#mvp_controller service
controller_service: controller
controller_state_service: controller/get_state

#state services
get_state_service: helm/get_state
get_states_service: helm/get_states
change_state_service: helm/change_state

```

- The Map window allows user to alter waypoint's latitude and longitude by dragging the **RED markers**. After user has finished waypoint dragging, user needs to click `Publish waypoints` which calls a ros service to update the waypoints using the service names defined in `config/gui_config.yaml` file.


```
##waypoints services
get_waypoints_service: helm/path_3d/get_next_waypoints
pub_waypoints_service: helm/path_3d/update_waypoints
```

- The map will also show the remaining waypoints in the helm in yellow dashline.

- Altitude for each section between two wayponts are displayed on the line. The current AUV altitude (depth) is also shown.

- A green tail after the AUV marker shows vehicle location in the past 20 seconds.