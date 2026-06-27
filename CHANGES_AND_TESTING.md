# Robot Localization and Navigation Fixes - Implementation Summary

**Date:** June 27, 2026

## Changes Implemented

### Phase 1: TF Tree Conflicts (CRITICAL)

#### ✅ 1. Disabled rf2o TF Publishing
**File:** `src/system_bringup/launch/robot_system.launch.py:214`
**Change:** `'publish_tf': True` → `'publish_tf': False`
**Why:** Prevents TF conflicts. EKF is now the single source of `custom_odom → custom_base_link` transform.

#### ✅ 2. Fixed EKF world_frame
**File:** `src/system_bringup/config/ekf.yaml:11`
**Change:** `world_frame: custom_odom` → `world_frame: map`
**Why:** Required for proper AMCL integration. AMCL publishes `map → custom_odom`, EKF publishes `custom_odom → custom_base_link`.

---

### Phase 2: EKF Sensor Fusion

#### ✅ 3. Enabled Encoder Position Data
**File:** `src/system_bringup/config/ekf.yaml:15-23`
**Changes:**
- Changed `odom0_config` to enable x, y position (was velocity-only)
- Changed `odom0_differential: true` → `false` (absolute pose)
- Changed `odom0_relative: true` → `false` (absolute measurements)
**Why:** Motor controller publishes absolute pose - now EKF uses it for better short-term accuracy.

#### ✅ 4. Fixed rf2o Differential Mode
**File:** `src/system_bringup/config/ekf.yaml:29`
**Change:** `odom1_differential: false` → `true`
**Why:** rf2o is scan-to-scan matching, producing differential odometry.

#### ✅ 5. Added Measurement Covariance Matrices
**File:** `src/system_bringup/config/ekf.yaml` (after line 41)
**Added:**
- `odom0_pose_covariance` - Encoder position trust (0.1m, 0.2rad)
- `odom0_twist_covariance` - Encoder velocity trust (0.1m/s, 0.1rad/s)
- `odom1_twist_covariance` - rf2o velocity trust (0.15m/s, 0.1rad/s)
**Why:** Tells EKF how much to trust each sensor, improving fusion quality.

---

### Phase 3: AMCL Tuning

#### ✅ 6. Optimized AMCL Parameters
**File:** `src/system_bringup/config/nav2_config.yaml:7-25`
**Changes:**
- `min_particles: 2000` → `500`
- `max_particles: 5000` → `2000`
- `update_min_d: 0.2` → `0.15` (more frequent updates)
- `update_min_a: 0.2` → `0.15` (more frequent updates)
- `resample_interval: 2` → `1` (more responsive)
- `alpha1-4: 0.2` → `0.1` (reduced process noise for better encoders)
**Why:** Reduces computational load, improves localization responsiveness, better tuned for your encoder quality.

---

### Phase 4: State Transition Improvements

#### ✅ 7. Improved Map Save Error Handling
**File:** `src/mode_manager_module/src/mode_manager.cpp:175-225`
**Changes:**
- Added error checking for map save - aborts transition if save fails
- Increased SLAM shutdown wait time (3s → 4s)
- Increased wait times for map_server and AMCL activation
- Added explicit wait after initial pose publication
**Why:** Prevents race conditions, ensures services are ready before use.

#### ✅ 8. Enhanced SLAM Launch Timing
**File:** `src/mode_manager_module/src/mode_manager.cpp:344-365`
**Changes:**
- Increased initial wait before SLAM launch (1s → 2s)
- Added 3-second wait after SLAM launch for initialization
- Added clarifying comments
**Why:** Gives SLAM time to properly initialize before receiving scan data.

---

## Expected TF Tree After Changes

```
map
 └── custom_odom              [AMCL publishes - localization mode only]
      └── custom_base_link    [EKF publishes - SINGLE source, fuses encoders + rf2o]
           ├── camera_link
           ├── lidar_front_left
           ├── lidar_front_right
           ├── lidar_back_left
           └── lidar_back_right
```

---

## Testing & Verification

### Step 1: Rebuild the Workspace

```bash
cd /home/hephaestus/Projects/Shopping-Cart-AMR-Diploma-Project
colcon build --packages-select mode_manager_module system_bringup
source install/setup.bash
```

### Step 2: Verify TF Tree (CRITICAL TEST)

**Test in localization mode:**
```bash
# Terminal 1: Launch with localization
ros2 launch system_bringup robot_system.launch.py slam_mode:=localization

# Terminal 2: Check TF tree
ros2 run tf2_tools view_frames
# Open frames.pdf and verify:
# - Only EKF publishes custom_odom → custom_base_link
# - No duplicate publishers
# - Clean tree: map → custom_odom → custom_base_link

# Terminal 3: Monitor TF publishers
ros2 topic echo /tf --no-arr | grep -A 5 "custom_odom"
# Should see: frame_id: "custom_odom", child_frame_id: "custom_base_link"
# Published by: ekf_filter_node ONLY
```

**Expected output:**
- `view_frames` should show ONE publisher for `custom_odom → custom_base_link`
- No TF warnings in terminal

### Step 3: Test EKF Sensor Fusion

```bash
# Terminal 1: Launch system
ros2 launch system_bringup robot_system.launch.py slam_mode:=localization

# Terminal 2: Monitor filtered odometry
ros2 topic echo /odometry/filtered

# Terminal 3: Compare with encoder odometry
ros2 topic echo /custom_odom_topic

# Terminal 4: Compare with rf2o odometry  
ros2 topic echo /odom_rf2o
```

**Expected behavior:**
- `/odometry/filtered` should be smooth, no jumps
- Should blend encoder position with rf2o corrections
- Drive robot in square pattern - minimal drift

### Step 4: Test AMCL Localization

```bash
# Terminal 1: Launch in localization mode
ros2 launch system_bringup robot_system.launch.py slam_mode:=localization rviz:=true

# In RViz:
# - Add display: /particle_cloud
# - Add display: /map
# - Verify particles converge quickly around robot
# - Use "2D Pose Estimate" to relocalize - should converge in 1-2 seconds

# Terminal 2: Monitor particle count
ros2 topic echo /particlecloud --field poses | wc -l
# Should be between 500-2000 particles
```

**Expected behavior:**
- Particles converge within 2-3 seconds after initialization
- Robot pose in RViz matches actual robot position
- Particles track robot smoothly when moving

### Step 5: Test State Transitions

**Test: IDLE → MAPPING → LOCALIZATION → NAVIGATION**

```bash
# Launch in idle mode
ros2 launch system_bringup robot_system.launch.py mode:=idle slam_mode:=mapping

# Switch to MAPPING mode (use your web interface or ROS service)
# Drive robot around to build map
# Monitor terminal for SLAM messages

# Save map and switch to localization
# (Use your mode_manager service call)
# Check terminal output:
# - "Harta salvata cu succes" (Map saved successfully)
# - "SLAM Toolbox oprit" (SLAM stopped)
# - "Activez AMCL" (AMCL activated)
# - "initial pose la (x, y, yaw)" (Initial pose published)

# Switch to NAVIGATION mode
# Send navigation goal via RViz
```

**Expected behavior:**
- Map saves without errors
- SLAM shuts down cleanly (4-second wait)
- AMCL activates successfully
- Initial pose is accepted
- Navigation goals work

### Step 6: Test Full Navigation

```bash
# Terminal 1: Launch in navigation mode
ros2 launch system_bringup robot_system.launch.py \
  mode:=navigation slam_mode:=localization rviz:=true

# Terminal 2: Monitor cmd_vel
ros2 topic echo /cmd_vel_nav2

# In RViz:
# - Set "2D Nav Goal"
# - Robot should navigate to goal
# - Monitor /local_costmap and /global_costmap
# - Check for obstacle avoidance
```

**Expected behavior:**
- Robot navigates smoothly to goals
- Avoids obstacles
- Doesn't get stuck
- Reaches goal within tolerance

---

## Troubleshooting

### Issue: TF warnings about multiple publishers

**Symptom:** `Warning: TF_REPEATED_DATA ignoring data with redundant timestamp`

**Solution:**
1. Check rf2o `publish_tf` is `False` in launch file
2. Check web_server `publish_odom_tf` parameter is `false`
3. Run `ros2 run tf2_tools view_frames` to identify duplicate publishers

### Issue: Robot doesn't localize properly

**Symptom:** AMCL particles don't converge, robot pose jumps

**Possible causes:**
1. **Bad map quality** - Rebuild map with better SLAM parameters
2. **Initial pose not set** - Check initial pose is published after AMCL activation
3. **EKF not fusing** - Check `/odometry/filtered` topic is publishing
4. **AMCL parameters too strict** - Try increasing `alpha1-4` values to 0.15

### Issue: EKF output has jumps

**Symptom:** `/odometry/filtered` has discontinuities

**Possible causes:**
1. **Encoder covariance too low** - Increase `odom0_pose_covariance` values
2. **rf2o scan matching failing** - Check lidar scans are clean, no noise
3. **TF conflicts still present** - Verify single TF publisher

### Issue: Navigation fails or robot gets stuck

**Symptom:** Robot stops moving, planning fails

**Possible causes:**
1. **Localization poor** - Fix AMCL first
2. **Costmap parameters** - Check inflation radius, obstacle detection
3. **Controller parameters** - Tune DWB critic weights
4. **TF tree broken** - Verify clean TF tree

---

## Performance Tuning (After Basic Functionality Works)

### If localization is too slow:
- Reduce `min_particles` to 300
- Increase `update_min_d` and `update_min_a` to 0.2

### If localization is too noisy:
- Increase `alpha1-4` to 0.15-0.2
- Increase particle count to 1000-3000

### If encoder drift is high:
- Reduce encoder trust: increase `odom0_pose_covariance` to 0.2
- Increase rf2o trust: reduce `odom1_twist_covariance` to 0.1

### If rf2o fails in open areas:
- Increase encoder trust: reduce `odom0_twist_covariance` to 0.05
- Consider adding IMU for angular velocity

---

## Next Steps (Optional Improvements)

### 1. Switch to MPPI Controller
For better obstacle avoidance and dynamic environments, consider switching from DWB to MPPI controller. See Nav2 documentation.

### 2. Add IMU Sensor
If angular velocity from encoders/rf2o is unreliable, add IMU as third odometry source in EKF.

### 3. Improve Costmap Configuration
Tune inflation radius and cost scaling for smoother paths. See Nav2 tuning guide.

### 4. Add Recovery Behaviors
Configure proper recovery behaviors for when robot gets stuck.

### 5. Optimize Scan Merger
If 4 lidars create too much data, consider downsampling or reducing scan frequency.

---

## Files Modified

1. **src/system_bringup/launch/robot_system.launch.py**
   - Line 214: Disabled rf2o TF publishing

2. **src/system_bringup/config/ekf.yaml**
   - Line 11: Fixed world_frame
   - Lines 15-23: Enabled encoder position
   - Line 29: Fixed rf2o differential mode
   - Lines 43+: Added measurement covariance matrices

3. **src/system_bringup/config/nav2_config.yaml**
   - Lines 7-25: Tuned AMCL parameters

4. **src/mode_manager_module/src/mode_manager.cpp**
   - Lines 175-225: Improved map save error handling
   - Lines 344-365: Enhanced SLAM launch timing

---

## Rollback Instructions

If you need to revert changes:

```bash
cd /home/hephaestus/Projects/Shopping-Cart-AMR-Diploma-Project
git status
git diff src/system_bringup/config/ekf.yaml
git diff src/system_bringup/config/nav2_config.yaml
git diff src/system_bringup/launch/robot_system.launch.py
git diff src/mode_manager_module/src/mode_manager.cpp

# To revert specific file:
git checkout HEAD -- <file_path>

# To revert all changes:
git reset --hard HEAD
```

---

## Support & References

**Nav2 Documentation:**
- AMCL Configuration: https://docs.nav2.org/configuration/packages/configuring-amcl.html
- EKF Configuration: http://docs.ros.org/en/humble/p/robot_localization/
- Tuning Guide: https://docs.nav2.org/tuning/index.html

**SLAM Toolbox:**
- GitHub: https://github.com/SteveMacenski/slam_toolbox
- Localization vs Mapping modes

**Key Concepts:**
- TF Tree hierarchy: map → odom → base_link
- AMCL provides map → odom transform (localization)
- EKF provides odom → base_link transform (sensor fusion)
- Only ONE node should publish each transform

---

**If you encounter issues not covered here, check:**
1. Terminal output for error messages
2. RViz warnings/errors
3. TF tree with `view_frames`
4. Topic outputs with `ros2 topic echo`

Good luck! Your robot should now have much better localization and navigation.
