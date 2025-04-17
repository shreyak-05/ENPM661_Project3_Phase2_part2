
### Part02 - Falconsim : Launching the ROS Node
Video Link : https://drive.google.com/file/d/1fLp9lNbKpIPAkeeVlLxAFEVBNpHiEHE1/view?usp=sharing

**Source ROS Environment:**  
Make sure your ROS environment is properly set up by running the following command

```bash
source /opt/ros/humble/setup.bash
```

Clone the repo 
```bash
cd ~/ros2_ws/src

git clone https://github.com/your‑username/ENPM661_Project3_Phase2_part2.git astar_falcon_planner
````
Launch the ROS Node:
Navigate to the Part02 directory and launch the ROS node using the provided launch file:
```bash
ros2 launch astar_falcon_planner ros_falcon_astar.launch.py \
    start_position:="[16.5, 5.5, 0.0]" \
    end_position:="[19.7, 8.4, 0.0]" \
    robot_radius:=0.22 \
    clearance:=0.05 \
    delta_time:=1.0 \
    goal_threshold:=0.3 \
    wheel_radius:=0.0335 \
    wheel_distance:=0.16 \
    rpms:="[50.0, 50.0]"
```

## Author 
 - Sagar Vijayakumar
   - UID : 116425438
   - sagvijay@umd.edu
       
 - Shreya Kalyanaraman
    - UID : 121166647
    - shreya05@umd.edu
         
 - Amogha Sunil
    - UID : 121098719
    - amoghats@umd.edu    
