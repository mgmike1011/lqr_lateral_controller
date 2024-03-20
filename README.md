# LQR lateral controller
<!-- Required -->
<!-- Package description -->

## Installation
<!-- Required -->
<!-- Things to consider:
    - How to build package? 
    - Are there any other 3rd party dependencies required? -->
```bash
cd ~/autoware/src/universe/autoware.universe/control
git clone https://github.com/mgmike1011/lqr_lateral_controller.git
cd ~/autoware
colcon build --packages-select lqr_lateral_controller --symlink-install
```
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
-DCMAKE_EXPORT_COMPILE_COMMANDS=On --packages-up-to f1tenth_launch
```
<!-- ```bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=On --packages-up-to lqr_lateral_controller
``` -->

## Usage
<!-- Required -->
<!-- Things to consider:
    - Launching package. 
    - Exposed API (example service/action call. -->
1. 

```bash
ros2 launch lqr_lateral_controller lqr_lateral_controller.launch.py
```

<!-- ## API -->
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->

<!-- ### Input

| Name         | Type                  | Description  |
| ------------ | --------------------- | ------------ |
| `topic_name` | std_msgs::msg::String | Sample desc. | -->

<!-- ### Output

| Name         | Type                  | Description  |
| ------------ | --------------------- | ------------ |
| `topic_name` | std_msgs::msg::String | Sample desc. |

### Services and Actions

| Name           | Type                   | Description  |
| -------------- | ---------------------- | ------------ |
| `service_name` | std_srvs::srv::Trigger | Sample desc. |

### Parameters

| Name         | Type | Description  |
| ------------ | ---- | ------------ |
| `param_name` | int  | Sample desc. | -->


## References / External links
<!-- Optional -->
### Agnieszka Piórkowska, Natalia Wiśniewska, Miłosz Gajewski
#### Politechnika Poznańska 2024
