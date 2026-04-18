# Final Year Project Report Draft

## Drafting Note
This English draft is reorganised to better match the **NUS School of Computing Final Year Project report style**: a concise abstract, hierarchical chapter structure, a focused main report, and appendices for project artefacts. The structure follows the guidance on the NUS Computing FYP report writing page and the NUS B.Comp Dissertation report format document, which state that the main report should be concise, clearly structured, and normally organised as *Introduction, main technical chapters, Conclusions, References, and Appendices*.

**Project Title**: Unified Integration and Comparative Evaluation of Four LiDAR-Inertial SLAM Algorithms for Livox MID360 Mobile Robot Navigation  
**Student**: [To be filled]  
**Programme / Department**: [To be filled]  
**University**: National University of Singapore  
**Supervisor**: [To be filled]  
**Academic Year**: [To be filled]  
**Project No.**: [To be filled if required]

---

## Abstract
This project develops a unified ROS-based navigation benchmark for a differential-drive mobile robot equipped with a Livox MID360 LiDAR and IMU. Four LiDAR-inertial SLAM algorithms, namely LIO-SAM, FAST-LIO, Point-LIO, and FASTER-LIO, are integrated into a common navigation stack built on `move_base`, DWA local planning, and local/global costmaps. The project makes two main contributions. First, it provides an engineering framework for standardised dataset recording, map generation, algorithm switching, rosbag-based static evaluation, and waypoint-based online navigation evaluation. Second, it studies how differences in odometry output, point-cloud representation, frame conventions, and scan generation affect downstream navigation behaviour under identical planning parameters. Static experiments on a common rosbag show that all four algorithms can localise successfully, but with different trade-offs in accuracy and computational cost. Online waypoint evaluation shows that LIO-SAM and FAST-LIO achieve full success on the archived benchmark route, while Point-LIO and FASTER-LIO required additional scan and body-frame fixes before stable costmap updates were obtained. The final outcome is a reusable benchmark platform and a documented set of engineering lessons for fair system-level comparison of LiDAR-inertial navigation algorithms on a real mobile robot.

**Subject Descriptors**: I.2.9 Robotics; C.4 Performance of Systems; I.4.8 Scene Analysis; I.5.4 Applications  
**Keywords**: LiDAR-inertial odometry, mobile robot navigation, Livox MID360, local costmap, ROS  
**Implementation Software and Hardware**: Ubuntu 20.04/18.04, ROS Noetic, Livox MID360, differential-drive mobile base, LIO-SAM, FAST-LIO, Point-LIO, FASTER-LIO, `move_base`, DWAPlannerROS, Evo

---

## Acknowledgements
This report summarises the implementation, debugging, and evaluation work completed on a real mobile robot platform during the Final Year Project. The author would like to thank the project supervisor for guidance, and the open-source communities behind LIO-SAM, FAST-LIO, Point-LIO, FASTER-LIO, and ROS navigation for making the experimental comparison possible.

---

# 1. Introduction

## 1.1 Background and Motivation
LiDAR-inertial odometry and SLAM have become core technologies for autonomous robots that operate in environments where GNSS is unreliable and visual conditions are inconsistent. For a real mobile robot, however, high-quality odometry alone is not sufficient. A complete autonomous navigation pipeline also requires consistent coordinate frames, map generation, obstacle perception, local costmap updates, motion planning, safe velocity control, and repeatable evaluation procedures.

The Livox MID360 is attractive for such a study because it combines a compact 3D LiDAR with an IMU and is practical for deployment on a small differential-drive robot. At the same time, the MID360 also exposes integration difficulties that are highly relevant for robotics engineering: non-repetitive scanning patterns, frame alignment issues, topic mismatches across SLAM packages, and local obstacle updates that may fail even when SLAM itself appears to work.

This project therefore focuses not only on running a single SLAM algorithm, but on building a **fair, reusable system-level benchmark** in which multiple LiDAR-inertial algorithms can be compared under the same robot, same map, same planning parameters, and same evaluation route.

## 1.2 Problem Statement
The four candidate algorithms, LIO-SAM, FAST-LIO, Point-LIO, and FASTER-LIO, differ in their odometry topics, point-cloud outputs, frame conventions, and computational characteristics. If they are connected to navigation naively, the comparison becomes unfair: one algorithm may publish a body-frame cloud, another a map-frame cloud; one may expose a dense local cloud, another only a sparse registered map cloud; one may provide a stable odometry topic, while another exposes a higher-frequency but less suitable intermediate estimate. As a result, downstream navigation performance may differ for reasons unrelated to the underlying SLAM quality alone.

The central problem addressed by this project is therefore:

**How can four heterogeneous LiDAR-inertial SLAM algorithms be integrated into a unified ROS navigation framework and evaluated fairly at both the trajectory level and the task-execution level?**

## 1.3 Objectives
The project objectives are:

1. To build a unified navigation interface for four LiDAR-inertial SLAM algorithms using a common ROS navigation stack.
2. To standardise rosbag recording, map generation, and multi-algorithm startup for repeatable experiments.
3. To construct both static rosbag evaluation and online waypoint navigation evaluation pipelines.
4. To identify and solve engineering issues that prevent fair comparison, especially those involving TF trees, scan generation, local costmap updates, and unstable process shutdown.
5. To produce a report-ready archive of data, scripts, maps, TF trees, videos, and evaluation summaries.

## 1.4 Contributions
The main contributions of the project are as follows:

1. A unified SLAM-to-navigation integration framework based on common topics `/slam/odom` and `/slam/cloud_registered`.
2. A practical toolchain for remote deployment, rosbag recording, map export, static trajectory evaluation, and online waypoint evaluation.
3. A documented debugging methodology for scan/costmap failures, TF inconsistencies, rosbag replay jitter, and process cleanup issues.
4. A benchmark dataset archive under the local `FYP` directory, including bags, generated maps, TF tree snapshots, obstacle-avoidance videos, and evaluation outputs.
5. A comparative analysis showing how upstream SLAM outputs affect downstream navigation behaviour even when the planner parameters are held constant.

---

# 2. Related Work

## 2.1 International Research on LiDAR SLAM and Navigation
Classical LiDAR odometry and mapping was strongly shaped by LOAM, which separated the front-end odometry and back-end mapping processes to achieve real-time performance with relatively low drift (Zhang and Singh, 2017). LOAM established the practical importance of using geometric features extracted from point clouds and remains a conceptual reference point for many later LiDAR-based systems.

For robot navigation, the Dynamic Window Approach (DWA) remains one of the most influential local planning methods. It searches directly in velocity space while respecting robot dynamics and obstacle constraints, and forms the conceptual basis of the `DWAPlannerROS` implementation used in this project (Fox, Burgard, and Thrun, 1997).

Among more recent LiDAR-inertial methods, LIO-SAM introduced a factor-graph formulation that tightly couples LiDAR odometry, IMU preintegration, and mapping, while allowing loop closure and multiple measurement sources to be fused consistently (Shan et al., 2020). LIO-SAM is especially influential because it balances real-time performance with graph-based smoothing and has become widely adopted in ROS-based research prototypes.

More broadly, recent reviews show that SLAM research has moved increasingly toward domain-aware and deployment-oriented systems, where practical concerns such as sensor configuration, computational load, and robustness in changing environments are as important as raw localisation accuracy (Yarovoi and Cho, 2024).

## 2.2 Regional and Domestic Research Status
Research groups in Hong Kong and mainland China have made especially strong contributions to tightly coupled LiDAR-inertial odometry in recent years. FAST-LIO2, developed by the University of Hong Kong, shifted away from hand-crafted feature extraction and instead registered raw points directly to the map using an iterated Kalman filter and incremental kd-tree map management, achieving strong efficiency and adaptability across sensor types (Xu et al., 2022).

Point-LIO further pushed the emphasis on update frequency and aggressive motion handling by introducing point-by-point state updates and a stochastic-process-based formulation for handling high-bandwidth motion and IMU saturation (He et al., 2023). In parallel, FASTER-LIO pursued computational efficiency through parallel sparse incremental voxels and lightweight tracking, aiming to retain accuracy while reducing runtime burden (Bai et al., 2022).

These works demonstrate that the current domestic and regional research trend is not only to improve localisation accuracy, but also to improve **update rate, robustness to aggressive motion, and suitability for real robotic platforms with limited computation**. This trend is directly relevant to the present project, which evaluates not just SLAM trajectories, but the effect of each algorithm on a full robot navigation stack.

## 2.3 Gap Addressed by This Project
Most algorithm papers evaluate localisation accuracy on public datasets and focus primarily on odometry or mapping metrics. By contrast, this project studies the **system-level gap** between SLAM outputs and real navigation behaviour. In particular, it asks how different choices of odometry topic, point-cloud topic, and frame convention influence:

1. scan generation,
2. local costmap updates,
3. obstacle clearance,
4. completion time, and
5. command smoothness.

This project therefore complements algorithm-level evaluation with a downstream navigation benchmark under a unified experimental platform.

---

# 3. System Overview

## 3.1 Hardware and Software Platform
The experimental platform is a small differential-drive mobile robot equipped with a Livox MID360 LiDAR/IMU. The software stack is based on ROS Noetic and a standard navigation architecture consisting of:

1. a static occupancy-grid map,
2. `pointcloud_to_laserscan` for obstacle observation generation,
3. `move_base` with global and local costmaps, and
4. `DWAPlannerROS` as the local planner.

The robot and workstation were operated in a multi-machine ROS configuration during real-world experiments, with the robot hosting the sensor and base drivers and the workstation used for visualisation, data analysis, and batch evaluation.

## 3.2 Unified Navigation Interface
A major engineering contribution of the project is the introduction of a common interface layer so that all four SLAM algorithms can be swapped without changing the navigation stack itself. The common interface consists of two topics:

1. `/slam/odom` for odometry input to navigation, and
2. `/slam/cloud_registered` for point-cloud input used to generate `/scan`.

The unification is performed through lightweight relay nodes and algorithm-specific startup logic. The current archived integration strategy is summarised in Table 1.

**Table 1. Unified interface used to connect four SLAM algorithms to the same navigation stack**

| Algorithm | Native odometry used | Native cloud used for navigation | Navigation scan frame |
|---|---|---|---|
| LIO-SAM | `/odometry/imu` | `/lio_sam/mapping/cloud_registered_raw` | `base_link` |
| FAST-LIO | `/Odometry` | `/cloud_registered` | `base_link` |
| Point-LIO | `/aft_mapped_to_init` | `/cloud_registered_body` | `base_link` |
| FASTER-LIO | `/Odometry` | `/cloud_registered_body` | `base_link` |

This design ensured that the navigation stack remained constant even while the upstream SLAM implementation changed.

## 3.3 TF Tree Harmonisation
The four algorithms do not expose identical TF trees or output topics. Therefore, the project standardised the downstream navigation assumptions around a common robot body frame, `base_link`, and a common global frame pair `map -> odom -> base_link`.

The archived baseline TF tree snapshots are stored under the `FYP/TF-tree` directory and include:

1. `tf_liosam_baseline.pdf`
2. `tf_fastlio_baseline.pdf`
3. `tf_pointlio_baseline.pdf`
4. `tf_fasterlio_baseline.pdf`

Local configuration inspection shows that the three filter-based algorithms already define `body_frame: base_link` and `map_frame: map` in their MID360 configuration files, while LIO-SAM currently uses `lidarFrame: base_link`, `baselinkFrame: base_link`, `odometryFrame: odom`, and `mapFrame: map`. The project did explore a more explicit `livox_frame` alignment for LIO-SAM, but that change was later rolled back to preserve a stable baseline for comparison.

The practical lesson is that fair comparison does not require all internal algorithm frames to be identical. It does, however, require a **stable downstream contract**: navigation must always know which odometry topic and which cloud topic to trust, and those topics must be consistent with the scan frame used by `pointcloud_to_laserscan` and the obstacle layer.

## 3.4 Obstacle Observation Chain
Obstacle handling in the navigation stack follows the chain below:

1. the selected SLAM point cloud is bridged to `/slam/cloud_registered`,
2. `pointcloud_to_laserscan` converts it into `/scan`,
3. the local obstacle layer consumes `/scan`, and
4. the local costmap is updated for DWA-based avoidance.

Much of the debugging effort in this project focused on this chain. In several cases, the SLAM algorithm itself was working, but the local costmap failed because the cloud was too sparse, in the wrong frame, or not appropriate for real-time 2D obstacle extraction.

---

# 4. Implementation and Engineering Toolchain

## 4.1 Recording and Dataset Management
A standardised data recording launch file was created to make real-world experiments repeatable. The recording pipeline performs three tasks at once:

1. smooths raw teleoperation commands before they reach the base controller,
2. publishes required static transforms and a unified odometry topic, and
3. records LiDAR, IMU, odometry, TF, and TF-static topics into rosbag files.

This standardisation was essential for later ATE/RPE computation, because early bags did not contain a suitable ground-truth topic.

## 4.2 Unified Startup Scripts
A unified startup script was developed so that each SLAM algorithm can be launched together with navigation using the same external interface. This script also performs cleanup of residual ROS nodes and relay processes so that repeated experiments do not inherit stale processes from previous runs. This was a necessary engineering step because process residue was repeatedly found to cause false failures, duplicated TFs, and costmap warnings.

## 4.3 Static Evaluation Scripts
The static evaluation pipeline automates the replay of a common rosbag for each SLAM algorithm, exports trajectories to TUM format, runs APE/RPE evaluation, and records resource usage and frame timing. The result directory structure contains per-algorithm logs, estimated trajectories, evaluation text outputs, and a machine-generated summary table.

## 4.4 Online Waypoint Evaluation Scripts
A separate waypoint evaluation tool publishes goals to the navigation stack and records task-level performance metrics such as success rate, total completion time, average speed, minimum scan distance, near-collision count, recovery count, command smoothness, and CPU/GPU usage. The evaluator also supports appending the robot start pose as a final goal, which makes return-to-start error measurable without manually duplicating the first waypoint.

## 4.5 Map Generation from PCD
The project includes a utility to convert 3D PCD maps into 2D occupancy maps without overwriting the default navigation map. This capability was used to generate new maps from saved point clouds while preserving a stable reference map for baseline experiments.

---

# 5. Experimental Methodology

## 5.1 Project Assets Reviewed for This Report
All currently archived FYP materials relevant to the report were reviewed. The local archive includes:

1. rosbags under `FYP/PointCloudBag`,
2. exported PCD map folders such as `maps`, `maps_324`, and `new_maps`,
3. TF tree snapshots under `FYP/TF-tree`,
4. online waypoint evaluation outputs under `FYP/nav_eval_results`,
5. static rosbag evaluation outputs under `FYP/static_eval_results`,
6. waypoint definitions under `FYP/waypoints`, and
7. obstacle-avoidance videos under `FYP/video`.

These artefacts form the empirical basis of the present report.

## 5.2 Static Rosbag Evaluation
The static evaluation uses a common recorded rosbag and runs each SLAM algorithm separately on identical sensor input. The outputs are converted to trajectory files and compared against the recorded reference odometry. The primary metrics are:

1. ATE RMSE,
2. RPE RMSE,
3. mean CPU usage,
4. mean memory usage,
5. mean frame time, and
6. odometry message count.

This evaluation isolates localisation and computational behaviour without the confounding influence of the navigation stack.

## 5.3 Online Waypoint Navigation Evaluation
The online navigation evaluation uses:

1. one common occupancy-grid map,
2. one common waypoint route in the `map` frame,
3. one common DWA and costmap configuration, and
4. one common robot platform.

The currently archived route contains four manually captured goals. The evaluator can append the start pose as a final goal, effectively creating a five-goal sequence with a return-to-start test.

The primary online metrics are:

1. success count and success rate,
2. total completion time,
3. total travelled path length,
4. average speed,
5. minimum scan distance to obstacles,
6. average goal position error,
7. average goal yaw error,
8. recovery count,
9. near-collision count,
10. command smoothness, and
11. system CPU/GPU usage.

## 5.4 Fairness Considerations
To keep the comparison as fair as possible, the project controlled the following variables across runs:

1. identical robot platform and sensor suite,
2. identical waypoint route,
3. identical navigation stack and planner configuration,
4. identical map-loading procedure, and
5. standardised odometry/cloud topic bridging.

At the same time, the report explicitly recognises that system-level comparison is inherently affected by the type of cloud and odometry each algorithm exposes. This is not a flaw in the study; it is part of the engineering reality being evaluated.

---

# 6. Experimental Results

## 6.1 Static Rosbag Evaluation Results
The latest complete static rosbag summary archived locally is shown in Table 2.

**Table 2. Static rosbag comparison of four algorithms**

| Algorithm | ATE RMSE (m) | RPE RMSE | CPU Mean (%) | Memory Mean (MB) | Mean Frame Time (s) | Odom Message Count |
|---|---:|---:|---:|---:|---:|---:|
| LIO-SAM | 0.141737 | 0.010850 | 57.855198 | 229.855216 | 0.005000 | 36682 |
| FAST-LIO | 0.142670 | 0.006296 | 33.503534 | 478.383343 | 0.100001 | 1837 |
| Point-LIO | 0.203076 | 0.021380 | 30.259501 | 203.800959 | 0.100006 | 1836 |
| FASTER-LIO | 0.144304 | 0.003444 | 122.935643 | 488.948487 | 0.100002 | 1837 |

These results suggest the following:

1. LIO-SAM, FAST-LIO, and FASTER-LIO achieve similar ATE on the archived bag, while Point-LIO shows visibly larger error.
2. FASTER-LIO obtains the lowest RPE among the four on this particular bag, suggesting very strong local consistency.
3. LIO-SAM publishes odometry at a much higher rate than the others in the archived setup, which explains its far higher odometry message count.
4. FASTER-LIO appears to achieve strong local accuracy at a significantly higher CPU cost.

## 6.2 Online Waypoint Navigation Results
The locally archived standardised online waypoint summaries currently cover LIO-SAM and FAST-LIO. These results are presented in Table 3.

**Table 3. Archived online navigation benchmark results**

| Metric | LIO-SAM | FAST-LIO |
|---|---:|---:|
| Goal count | 5 | 5 |
| Success count | 5 | 5 |
| Success rate | 1.000 | 1.000 |
| Total time (s) | 123.551 | 120.301 |
| Total path length (m) | 29.568 | 29.426 |
| Average speed (m/s) | 0.239 | 0.245 |
| Minimum scan distance (m) | 0.658 | 0.309 |
| Total recovery count | 0 | 0 |
| Average goal position error (m) | 0.237 | 0.241 |
| Average goal yaw error (rad) | 1.320 | 1.301 |
| Average CPU (%) | 12.559 | 12.448 |
| Average GPU (%) | 13.164 | 15.475 |
| Average command linear std | 0.0286 | 0.0222 |
| Average command angular std | 0.1595 | 0.1045 |
| Return-to-start position error (m) | 0.240 | 0.243 |
| Return-to-start yaw error (rad) | 0.266 | 0.151 |

Several observations follow from these results:

1. Both archived runs achieved full waypoint success.
2. FAST-LIO was slightly faster overall on the archived route.
3. LIO-SAM maintained a much larger minimum obstacle distance in the archived run, suggesting more conservative clearance on that route.
4. FAST-LIO produced smoother command traces in the archived benchmark according to both linear and angular command standard deviation.

## 6.3 Status of Point-LIO and FASTER-LIO in Online Navigation
For Point-LIO and FASTER-LIO, the present local archive contains complete static evaluation outputs and TF tree baselines, but not yet a complete set of stable online waypoint `summary.json` files comparable to the LIO-SAM and FAST-LIO runs. However, the debugging process already established the main causes of their earlier online failures:

1. Point-LIO originally did not expose a usable body-frame cloud for navigation until `scan_bodyframe_pub_en` was enabled and the navigation bridge was switched to `/cloud_registered_body`.
2. FASTER-LIO initially used a world-frame cloud to generate `/scan`, which introduced noticeable lag in local costmap updates; this was improved by switching to `/cloud_registered_body`.

The project therefore reached a practically useful state in which all four algorithms could be brought into a common navigation framework, while the fully archived online numerical comparison of Point-LIO and FASTER-LIO remains a clear next step.

## 6.4 Qualitative Evidence: TF Trees, Videos, and Generated Maps
The report is also supported by qualitative artefacts archived under `FYP`, including:

1. baseline TF tree PDFs for all four algorithms,
2. obstacle-avoidance videos recorded during integration and testing, and
3. generated PCD maps and 2D occupancy maps.

These artefacts were particularly useful when numerical logs alone did not explain phenomena such as a visible point cloud without a corresponding local costmap update, or large directional differences in navigation speed despite identical DWA parameters.

---

# 7. Major Engineering Problems and Resolutions

## 7.1 Abrupt Teleoperation and Poor Recording Quality
**Problem.** Early manual driving produced large velocity jumps and unstable motion, which degraded dataset quality and made repeatable evaluation difficult.

**Cause.** Raw teleoperation commands were sent directly to the base without acceleration limiting.

**Resolution.** A command smoother was implemented to enforce linear and angular acceleration/deceleration limits before publishing `/cmd_vel`. Conservative defaults were added to the standard recording launch.

**Outcome.** Robot motion during dataset acquisition became smoother and easier to reproduce.

## 7.2 Missing Ground Truth for ATE/RPE
**Problem.** Early offline evaluation produced `NA` for ATE and RPE.

**Cause.** The original recording pipeline did not preserve a usable reference odometry topic.

**Resolution.** The standard recording launch was revised to include `/odom` alongside LiDAR, IMU, TF, and TF-static topics.

**Outcome.** Later static evaluation runs successfully produced valid ATE and RPE values for all four algorithms.

## 7.3 Residual ROS Processes Breaking Repeated Runs
**Problem.** Re-running SLAM and navigation often left behind relay or launch processes that kept printing warnings, respawning nodes, or interfering with later runs.

**Cause.** ROS child processes and relay nodes were not being shut down consistently.

**Resolution.** The unified startup script was expanded to clean process groups, kill matching relays and navigation processes, and run ROS node cleanup.

**Outcome.** Repeated experiments became much more stable and debugging became significantly easier.

## 7.4 LIO-SAM Map Export Crash on Empty Clouds
**Problem.** LIO-SAM could crash during map saving with an exception indicating that the input point cloud had no data.

**Cause.** The map-saving path did not guard against empty keyframe sets or empty global maps.

**Resolution.** A source-level guard was added in `mapOptmization.cpp` to skip saving when the relevant point sets were empty.

**Outcome.** Map export no longer crashed on empty-map cases.

## 7.5 Local Costmap Not Updating Even When the Scan Was Visible
**Problem.** During navigation, `/scan` could be visible in RViz while the local costmap still failed to update.

**Cause.** This turned out to be a chain-level problem rather than a single configuration error. The main causes identified were: wrong cloud topics, world-frame clouds being reused for near-field obstacle generation, missing body-frame cloud publication, and frame mismatches between SLAM output and costmap expectations.

**Resolution.** The navigation bridge was standardised so that each algorithm exposed an appropriate odometry topic and an appropriate cloud for obstacle generation. Point-LIO and FASTER-LIO were particularly improved by switching navigation input to `/cloud_registered_body`.

**Outcome.** Local costmap behaviour became much more reliable, especially for Point-LIO and FASTER-LIO.

## 7.6 Point-LIO and FASTER-LIO Scan Generation Failures
**Problem.** Point-LIO and FASTER-LIO sometimes produced empty or severely delayed `/scan` output for navigation.

**Cause.** The initial setup treated them as if they could be bridged identically to FAST-LIO, but their useful cloud outputs for navigation were not identical in practice. Point-LIO initially lacked a usable body-frame cloud in the tested configuration, and FASTER-LIO exhibited delay when a world-frame registered cloud was used to regenerate a local scan.

**Resolution.** Body-frame cloud publication was enabled where necessary, and the bridge was updated to use `/cloud_registered_body` instead of a registered map-frame cloud.

**Outcome.** Scan availability and local costmap responsiveness improved substantially.

## 7.7 LIO-SAM Rosbag Replay Jitter
**Problem.** LIO-SAM appeared to "shake" or jump when replaying rosbag data.

**Cause.** The replayed bag contained `/tf` and related topics that conflicted with the live TF chain published by the currently running SLAM and robot-state publisher nodes.

**Resolution.** Replay was restricted to the raw sensor topics required by LIO-SAM, while avoiding simultaneous replay of conflicting TF data.

**Outcome.** Offline playback became much more stable and interpretable.

## 7.8 Online Evaluation Script Failures and Logging Issues
**Problem.** The waypoint evaluation script initially failed to create log output directories and occasionally sent goals that were not properly consumed by navigation.

**Cause.** The output directory was not always created before the first log write, and goal publication assumed an already-stable subscriber connection.

**Resolution.** The evaluator was extended to create output directories automatically, record richer event logs and timelines, and verify goal-subscriber readiness before evaluation.

**Outcome.** Online navigation experiments became easier to diagnose and reproduce.

## 7.9 Direction-Dependent Navigation Behaviour Under Identical DWA Parameters
**Problem.** The robot could move much faster in one world direction than the opposite direction even though the planner parameters were unchanged and the robot still moved forward in both cases.

**Cause.** The effect was traced not to DWA using different parameters, but to SLAM-dependent differences in odometry smoothness, cloud density, and obstacle representation. In particular, LIO-SAM and FAST-LIO exposed different odometry and cloud characteristics to the same navigation stack.

**Resolution.** The study shifted from treating the planner as the only cause to analysing the upstream odometry and cloud pipeline as part of the navigation system.

**Outcome.** This became an important conceptual finding of the project: **same planner parameters do not imply same navigation behaviour when the perception and odometry inputs differ**.

---

# 8. Discussion

## 8.1 Why the Same Planner Parameters Produced Different Robot Behaviour
One of the key lessons from this project is that a fair planner comparison requires much more than identical DWA parameters. The planner responds to the state estimate and obstacle observations it receives. Therefore, if one SLAM algorithm publishes denser clouds, more conservative near-field observations, or noisier odometry, the robot may slow down earlier or take wider detours even though the planner configuration file is unchanged.

This is precisely what was observed in the project. FAST-LIO, Point-LIO, and FASTER-LIO belong to a similar tightly coupled filter-based family, but they still needed different cloud selections to produce usable local scans. LIO-SAM, meanwhile, interacted differently with the navigation stack depending on whether the odometry source and cloud source were taken from its intermediate or mapping outputs. The project therefore shows that system-level navigation performance is a function of the **whole perception-to-planning chain**, not only of the planner itself.

## 8.2 Engineering Fairness Versus Algorithm Purity
A second lesson concerns fairness. If every algorithm is forced into exactly the same cloud topic type regardless of its intended design, the comparison may become formally uniform but practically meaningless. In this project, fairness was defined instead as follows:

1. same robot,
2. same map,
3. same waypoint task,
4. same navigation stack,
5. same evaluation metrics,
6. but a carefully justified and documented choice of odometry and cloud topic for each algorithm.

This is a more realistic definition of fairness for system integration work.

## 8.3 Limits of the Current Archive
The local project archive already supports a strong report with complete static comparisons and a partially complete online comparison. However, two limitations remain:

1. the standardised online quantitative archive is currently complete for LIO-SAM and FAST-LIO, but not yet equally complete for Point-LIO and FASTER-LIO; and
2. some qualitative evidence exists only as video or debugging history rather than as a final benchmark table.

These limitations do not invalidate the work, but they should be stated clearly in the final report.

---

# 9. Conclusion and Future Work

This project developed a unified benchmark platform for comparing four LiDAR-inertial SLAM algorithms on a Livox MID360 mobile robot under a shared ROS navigation stack. The work went beyond simply launching open-source packages: it established a full engineering workflow for recording, deployment, unified startup, static rosbag evaluation, online waypoint evaluation, map generation, and problem diagnosis.

The completed experiments show that the four algorithms differ not only in localisation accuracy and computational load, but also in how well their outputs support real-time local obstacle perception and navigation. LIO-SAM and FAST-LIO already achieved stable archived online waypoint results under the current framework, while Point-LIO and FASTER-LIO required additional body-frame cloud fixes to become suitable for the same navigation setup. The project also demonstrated that many apparent navigation problems are in fact integration problems involving coordinate frames, cloud representations, and observation pipelines rather than failures of the planner alone.

Future work should focus on three directions. First, Point-LIO and FASTER-LIO should be rerun through the full archived online benchmark so that the final report contains all four standardised online result tables. Second, the navigation configuration should be further refined using more realistic robot footprint calibration and additional obstacle scenarios. Third, the benchmark can be extended with repeated trials, statistical confidence intervals, and more diverse route layouts so that the comparison becomes publication-ready.

---

# References

Bai, C., Xiao, T., Chen, Y., Wang, H., Zhang, F., & Gao, X. (2022). *Faster-LIO: Lightweight tightly coupled lidar-inertial odometry using parallel sparse incremental voxels*. IEEE Robotics and Automation Letters, 7(2), 4861-4868. https://doi.org/10.1109/LRA.2022.3152830

Fox, D., Burgard, W., & Thrun, S. (1997). *The Dynamic Window Approach to Collision Avoidance*. IEEE Robotics & Automation Magazine, 4(1). https://www.cs.cmu.edu/~dfox/abstracts/colli-ieee.abstract.html

He, D., Xu, W., Chen, N., Kong, F., Yuan, C., & Zhang, F. (2023). *Point-LIO: Robust High-Bandwidth Light Detection and Ranging Inertial Odometry*. Advanced Intelligent Systems, 5(7), Article 2200459. https://doi.org/10.1002/aisy.202200459

National University of Singapore, School of Computing. (n.d.). *BComp Dissertation (FYP) - Report Writing*. https://www.comp.nus.edu.sg/programmes/ug/project/fyp/report/

National University of Singapore, School of Computing. (2023). *FYP Report Format*. https://www.comp.nus.edu.sg/wp-content/uploads/2023/10/FYP-Report-Format-final_000.pdf

Shan, T., Englot, B., Meyers, D., Wang, W., Ratti, C., & Rus, D. (2020). *LIO-SAM: Tightly-coupled lidar inertial odometry via smoothing and mapping*. In 2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS) (pp. 5135-5142). https://doi.org/10.1109/IROS45743.2020.9341176

Xu, W., Cai, Y., He, D., Lin, J., & Zhang, F. (2022). *FAST-LIO2: Fast Direct LiDAR-Inertial Odometry*. IEEE Transactions on Robotics, 38(4), 2053-2073. https://doi.org/10.1109/TRO.2022.3141876

Yarovoi, A., & Cho, Y. K. (2024). *Review of simultaneous localization and mapping (SLAM) for construction robotics applications*. Automation in Construction, 162, 105344. https://doi.org/10.1016/j.autcon.2024.105344

Zhang, J., & Singh, S. (2017). *LOAM: Lidar odometry and mapping in real-time*. Autonomous Robots, 41, 401-416.

---

# Appendix A. Archived Project Assets Reviewed for This Report

The following artefacts were reviewed while drafting this report.

## A.1 Rosbags and PCD Maps
The `FYP/PointCloudBag` archive contains several recorded bags, including evaluation bags from March and April 2026, as well as exported LIO-SAM point-cloud maps such as `GlobalMap.pcd`, `SurfMap.pcd`, `CornerMap.pcd`, and associated map folders.

## A.2 TF Tree Snapshots
The `FYP/TF-tree` archive contains baseline TF tree PDFs for:

1. LIO-SAM,
2. FAST-LIO,
3. Point-LIO, and
4. FASTER-LIO.

These snapshots were used to cross-check frame-level integration assumptions.

## A.3 Online Evaluation Outputs
The `FYP/nav_eval_results` archive currently contains standardised online evaluation folders for:

1. `manual_liosam`, and
2. `manual_fastlio`.

Each folder includes:

1. `summary.json`,
2. `summary.txt`,
3. `per_goal.csv`,
4. `events.log`, and
5. per-goal timeline CSV files.

## A.4 Static Evaluation Outputs
The `FYP/static_eval_results` archive contains multi-algorithm rosbag evaluation folders with:

1. trajectory exports,
2. APE/RPE reports,
3. resource-usage logs,
4. frame-time CSVs, and
5. `summary.md` / `summary.csv` files.

## A.5 Videos and Qualitative Evidence
The `FYP/video` archive contains obstacle-avoidance and algorithm-analysis recordings, including generic avoidance demonstrations and algorithm-specific clips such as the FAST-LIO-labelled recording.

---

# Appendix B. Project Scripts and Their Roles

To keep the main report concise, code is not reproduced here. Instead, the principal scripts are summarised below.

| Script or file | Main role |
|---|---|
| `record_dataset.launch` | standardised rosbag recording with odometry bridging and command smoothing |
| `cmd_vel_smoother.py` | limits acceleration and deceleration of teleoperation commands |
| `run_slam_nav.sh` | unified SLAM + navigation startup and cleanup |
| `eval_static_rosbag.sh` | batch static rosbag evaluation driver |
| `slam_eval_monitor.py` | resource and frame-time monitoring during static evaluation |
| `bag_to_tum.py` | converts ROS messages into TUM trajectory format |
| `summarize_static_eval.py` | produces static comparison summary tables |
| `capture_waypoints.py` | records waypoint routes in the map frame |
| `nav_waypoint_eval.py` | single-algorithm online waypoint evaluation |
| `eval_nav_4alg.sh` | batch online evaluation across four algorithms |
| `summarize_nav_eval.py` | merges navigation results into summary tables |
| `pcd_to_grid.py` | converts PCD map files into 2D occupancy maps |

---

# Appendix C. Suggested Final Editing Before Submission

Before converting this draft into the final submitted report, the following polishing steps are recommended:

1. replace title-page placeholders with the official NUS information required by the department,
2. add selected figures from RViz, TF trees, and map outputs,
3. add screenshots or still frames from obstacle-avoidance videos,
4. rerun and archive standardised online results for Point-LIO and FASTER-LIO, and
5. convert local directory references into figure numbers, table numbers, or appendix notes as needed.
