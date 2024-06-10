# Vehicle Status page
This page display vital messages from the vehicle.
The information is obtained from rostopics that is defined in `config/gui_config.yaml`

```
name_space: alpha_rise

##vehicle status
poses_source: odometry/filtered/local
geo_pose_source: odometry/geopose
vitals_source: power_monitor/power
```
- Again the topic is from the machine that is running mvp_gui
- Negative altitude means below sea surface.
- Vehicle pose frame_id is from the `odometry` messages
- The simple plot at the botton showing vehicle heading relative to North and East.
