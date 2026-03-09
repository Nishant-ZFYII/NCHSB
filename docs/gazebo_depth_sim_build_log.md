# Gazebo Depth-Error Simulation -- Build Log

**Project:** TIER 2 closed-loop Gazebo simulation study for RA-L paper  
**Branch:** `feat/gazebo-depth-sim` (both NCHSB and ml_pipeline)  
**Started:** 2026-02-27  
**Plan file:** `~/.cursor/plans/gazebo_closed-loop_sim_6fcc38fb.plan.md`

---

## Log Format

Each entry records:
- **Phase / Task** -- which plan phase this belongs to
- **What was done** -- action taken, files created/modified
- **Commands run** -- exact ROS/build/git commands
- **Output / Result** -- what happened, key numbers
- **Errors & Fixes** -- any issues and how they were resolved

---

## Phase 0: Branch Setup

**Date:** 2026-02-27  
**Goal:** Create isolated `feat/gazebo-depth-sim` branches on both repos so Tier 2 work does not contaminate existing branches.

### Step 0.1: Pre-branch cleanup -- NCHSB

Both repos had uncommitted changes on their working branches. Committed them first to ensure clean branch points.

**NCHSB** (`feat/corridor-social-nav`):
- 148 files staged (modified configs, new SLAM screenshots, ablation launch files, replayer nodes, build log updates)
- Added `rosbags/` to `.gitignore` (8.1GB directory, too large for git)
- `.gitignore` already excluded `build/`, `install/`, `log/`, `*.db3`, `*.bag`, `aws-robomaker-hospital-world/`

**Command:**
```bash
cd ~/MS_Project/NCHSB
# Added rosbags/ to .gitignore
git add -A
git commit -m "Pre-branch snapshot: SLAM screenshots, ablation configs, replayers, and build log updates"
git push origin feat/corridor-social-nav
```

**Result:** Commit `9f46b8f`, pushed successfully.

### Step 0.2: Pre-branch cleanup -- ml_pipeline

**ml_pipeline** (`v3-da3-efficientvit`):
- 1 file modified: `changelog.tex` (+251 lines -- V6/V6.2/V8/V9 corridor evals, Vivek's HPC training runs)

**Command:**
```bash
cd ~/MS_Project/ml_pipeline
git add changelog.tex
git commit -m "Update changelog: V6/V6.2/V8/V9 corridor evals, Vivek's HPC training runs"
git push origin v3-da3-efficientvit
```

**Result:** Commit `6af35d0`, pushed successfully.

### Step 0.3: Create feature branches

**Commands:**
```bash
# NCHSB
cd ~/MS_Project/NCHSB
git checkout -b feat/gazebo-depth-sim
git push -u origin feat/gazebo-depth-sim

# ml_pipeline
cd ~/MS_Project/ml_pipeline
git checkout -b feat/gazebo-depth-sim
git push -u origin feat/gazebo-depth-sim
```

**Result:**
- NCHSB: Branch `feat/gazebo-depth-sim` created from `9f46b8f` (parent: `feat/corridor-social-nav`), pushed to `origin`
- ml_pipeline: Branch `feat/gazebo-depth-sim` created from `6af35d0` (parent: `v3-da3-efficientvit`), pushed to `origin`

**Verification:**
```
NCHSB:      9f46b8f on feat/gazebo-depth-sim, tracking origin/feat/gazebo-depth-sim
ml_pipeline: 6af35d0 on feat/gazebo-depth-sim, tracking origin/feat/gazebo-depth-sim
```

### Step 0.4: Build log created

**File:** `NCHSB/docs/gazebo_depth_sim_build_log.md` (this file)

Tracks every action, command, file, output, and error for the entire Gazebo depth-error simulation study.

### Errors & Fixes

None. Clean branch creation.

---

## Phase 0 Status: COMPLETE

**Summary:**
- Both repos on `feat/gazebo-depth-sim` branch
- Parent branches committed and pushed (no dirty state left behind)
- Build log initialized
- Ready to proceed to Phase 1: Add RGBD Camera to Robot

---

## Phase 1: Add RGBD Camera to Robot

**Date:** (pending)  
**Goal:** Add an RGBD camera sensor to the simulated robot matching the real Orbbec Femto Bolt specs (640x480, 15Hz, 0.1-5.0m). Bridge camera topics to ROS2.

### Files to modify:
- `src/rc_model_description/urdf/links.xacro` -- add `camera_link`, `camera_color_optical_frame`, RGBD sensor
- `src/rc_model_description/urdf/joints.xacro` -- add `camera_joint` (fixed, from `chassis_link`)
- `src/rc_model_description/launch/fortress_bringup.launch.py` -- bridge `/camera/*` topics

### Checkpoint 1 (Manual Test):
```bash
colcon build --packages-select rc_model_description
ros2 launch rc_model_description fortress_bringup.launch.py \
    world:=corridor_narrow.sdf spawn_x:=-6.5 spawn_y:=0.0 spawn_yaw:=0.0
```
- [ ] `/camera/color/image_raw` publishes
- [ ] `/camera/depth` publishes at ~15Hz
- [ ] `/scan` (LiDAR) still works
- [ ] RViz2 shows RGB + depth images
- [ ] Robot spawns correctly

(entries will be added as work proceeds)
