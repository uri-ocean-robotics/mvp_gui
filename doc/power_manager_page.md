# Power manager page
This page is used to call the service and publish message to the topics to [alpha_gpio_manager](https://github.com/uri-ocean-robotics/alpha_core) which runs on AUV computer and controls GPIOs.

-The table will list all the services available for setting device power.
The list is generated automatically from the response messsage from the service defined in `config/gui_config.yaml` file.
```
get_power_port_srv: gpio_manager/get_power_status
```

- Status = 1 means the MOSFET will be on

- When `switch` button is clicked, the backend software will alter a data entry in the databse, such that the ros part of the software is awared and to call the rosservice for changing the MOSFET state.

- `Lumen brightness` is used to alter the brightness of underwater lights (BlueRobotics Lumen) via PWM signals. The pwm duty cycle has a range from 1 ms to 1.9ms corresponding to 0 to 100 brightness for the Lumen LED.
The GUI will set the value to 1.00 by default when started.
Each increment is 0.1 ms.