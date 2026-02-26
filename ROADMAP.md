# Roadmap to Self-Assembling Space Robotics

A progressive series of projects building from a 6-DOF robotic arm to autonomous self-assembling robotics in space environments. Each phase builds on the skills and systems developed in previous phases.

---

## Phase 1: Foundations

### 1. 6-DOF Robotic Arm IK + Vision (Done)

**How this works toward the end goal:**
Every self-assembling space robot needs to physically manipulate objects — pick up a structural module, orient it correctly, and place it precisely. This project establishes the foundational manipulation pipeline: perceive an object's location with computer vision, compute the joint angles needed to reach it via inverse kinematics, execute a smooth trajectory, and grip/release at the right moment. The 10-step state machine (approach → descend → grip → lift → traverse → descend → release → retract → return → complete) is the same fundamental sequence that a space assembly robot performs when placing a truss segment or connecting a module. Without reliable single-arm pick-and-place, nothing else in the roadmap is possible.

**Skills and tools — why they matter:**
- **C++ (Arduino firmware):** Embedded C/C++ is the language of real-time robot control. Space robots run on radiation-hardened processors with limited resources — efficient, low-level code is mandatory. Writing IK solvers and servo interpolation in C++ builds fluency with the same language used in flight software for robotic systems like the Canadarm2 on the ISS.
- **Python:** The high-level glue language of robotics. Used for vision pipelines, simulation scripting, data analysis, and prototyping algorithms before porting them to C++. Nearly every robotics research lab uses Python for rapid development.
- **OpenCV:** The standard computer vision library. Object detection via HSV filtering and contour analysis is foundational — the same principles (segment, identify, localize) scale up to detecting modules floating in space using stereo cameras or depth sensors. Understanding color spaces, image transformations, and coordinate frame conversions is essential.
- **Arduino:** Teaches the fundamentals of embedded systems — reading sensors, driving actuators, handling serial communication, managing timing loops. These concepts transfer directly to more powerful embedded platforms (STM32, Jetson) used in real robotic systems.
- **Inverse kinematics (geometric approach):** The mathematical core of robotic manipulation. Understanding how to convert a desired end-effector position into joint angles using trigonometry, atan2, and the law of cosines builds the intuition needed for more advanced IK methods (Jacobian-based, numerical optimization) used in higher-DOF systems and multi-robot coordination.

---

### 2. Path Planning (RRT/A*)

**How this works toward the end goal:**
In space assembly, robots can't just move in straight lines — they must navigate around the structure they're building, avoid other robots, and dodge tethers or solar panels. Path planning algorithms compute collision-free trajectories through complex 3D environments. A* provides optimal paths on discretized grids (useful for mobile base navigation), while RRT and RRT* handle high-dimensional configuration spaces (a 6-DOF arm has a 6D configuration space where each dimension is a joint angle). Space assembly planners must find paths that are not only collision-free but also energy-efficient — minimizing fuel consumption is critical when every gram of propellant matters. This project builds the algorithmic foundation for all future trajectory generation.

**Skills and tools — why they matter:**
- **Algorithm design and analysis:** Path planning is fundamentally a search problem. Understanding tree-based search (RRT), graph search (A*), and their optimality guarantees (RRT* converges to optimal) builds the computer science foundation needed for task planning, assembly sequence planning, and multi-robot coordination algorithms in later phases.
- **Configuration space (C-space) reasoning:** Thinking about robot motion in joint space rather than Cartesian space is a conceptual leap that unlocks all advanced motion planning. In C-space, the robot is a point and obstacles become complex shapes — this abstraction is how real robotic systems plan motions.
- **Collision detection:** Checking whether a robot configuration collides with obstacles requires computational geometry (bounding boxes, GJK algorithm, signed distance fields). Fast collision checking is essential for real-time replanning in dynamic environments — a capability every space assembly robot needs.
- **Python / C++:** Prototyping planners in Python for rapid iteration, then implementing performance-critical components in C++ for real-time execution. This dual-language workflow is standard in robotics research and industry.
- **Visualization:** Rendering search trees, planned paths, and obstacle maps builds skills in scientific visualization that are essential for debugging complex robotic behaviors and presenting research results.

---

### 3. Force/Torque Sensing & Compliance

**How this works toward the end goal:**
Pure position control (move to X,Y,Z) fails when parts must be pressed together, slid into slots, or snapped into place. Assembly requires force control — the ability to push with a specific force, detect contact, and respond to unexpected resistance. In space, this is even more critical: without gravity to seat components, robots must apply precise forces to dock modules while compensating for the free-floating dynamics (pushing too hard sends both the robot and the module tumbling). Impedance control — making the robot behave like a spring-damper system at the end effector — is the standard approach for safe, compliant interaction and is used on every space manipulator from the Canadarm2 to the European Robotic Arm.

**Skills and tools — why they matter:**
- **Control theory (impedance/admittance control):** The mathematical framework for specifying how a robot should respond to forces. Understanding mass-spring-damper models, transfer functions, and stability criteria is essential for designing controllers that work in the challenging dynamics of space (no friction, no gravity, flexible structures).
- **Sensor integration (force-sensitive resistors, load cells):** Reading analog sensors, filtering noisy signals (low-pass, Kalman), and fusing sensor data with motor commands teaches the sensor-actuator integration that every real robot requires. Space robots use multi-axis force/torque sensors at their wrists — understanding the principles starts here.
- **Embedded real-time control:** Force control loops must run at high frequencies (500Hz–1kHz) to maintain stability. Implementing these on embedded hardware teaches real-time programming constraints — deterministic timing, interrupt handling, buffer management — all critical for flight software.
- **Contact modeling:** Understanding how forces arise during contact (Hertz contact, friction cones, stick-slip transitions) informs both controller design and simulation fidelity. Accurate contact models are one of the hardest problems in space assembly simulation.

---

## Phase 2: Simulation & Advanced Control

### 4. MuJoCo or Gazebo Sim Setup

**How this works toward the end goal:**
You cannot test space assembly algorithms on real hardware orbiting Earth during development. High-fidelity simulation is the primary development platform for space robotics — NASA/JPL uses Gazebo and custom simulators extensively. A simulation environment lets you iterate on control algorithms 1000x faster than hardware, test failure scenarios safely, train reinforcement learning policies, and validate entire mission sequences before committing to a flight design. Establishing a robust sim-to-real pipeline (where controllers developed in simulation transfer to real hardware with minimal tuning) is a skill that directly applies to space robotics development, where the "real" system might not exist until years into the project.

**Skills and tools — why they matter:**
- **MuJoCo:** The gold standard for contact-rich manipulation simulation. Its fast, accurate contact solver makes it ideal for assembly tasks where parts must be pressed, slid, and snapped together. MuJoCo is also the standard platform for reinforcement learning in robotics. Understanding MJCF (MuJoCo's model format), its constraint solver, and its Python API (via `mujoco` or `dm_control`) opens the door to state-of-the-art manipulation research.
- **Gazebo + ROS2:** The standard simulation platform for multi-robot systems and full-system integration. ROS2 provides the middleware (pub/sub messaging, transforms, parameter servers) that connects perception, planning, and control. Gazebo provides physics simulation with sensor plugins (cameras, LiDAR, IMU). Understanding the ROS2 ecosystem is expected in virtually every robotics job and research lab.
- **URDF/MJCF modeling:** Describing a robot's kinematics (joint types, link lengths) and dynamics (masses, inertias, friction) in a structured format. Accurate models are essential for simulation fidelity. This skill transfers directly to modeling space robots and the structures they assemble.
- **Physics simulation fundamentals:** Understanding time-stepping, integration methods (semi-implicit Euler, Runge-Kutta), constraint solving, and numerical stability. These concepts are critical for building or modifying simulators for specialized environments like microgravity.
- **Sim-to-real transfer:** Techniques like domain randomization (varying friction, mass, sensor noise in sim), system identification (measuring real-world parameters to match sim), and progressive transfer (sim → lab → field) are active research areas directly relevant to space robotics where real testing is extremely limited.

---

### 5. Simulated Assembly (snap-fit blocks)

**How this works toward the end goal:**
This project makes the leap from moving individual objects to building structures — the core task of self-assembling space robotics. Decomposing a target structure into an ordered sequence of assembly operations is a planning problem: which block goes first? Can you place block C before block B without the structure collapsing? In space, assembly sequence planning is even more constrained — you must consider which configurations are structurally stable without gravity, which placements are reachable given the robot's current position, and how to minimize the number of robot repositions. This project builds the assembly planning skills that scale directly to space structure construction.

**Skills and tools — why they matter:**
- **Task and assembly planning:** Decomposing a high-level goal ("build this structure") into a sequence of primitive actions ("pick block A, place at position X"). This requires reasoning about dependencies, ordering constraints, and structural stability. The same planning frameworks (STRIPS, PDDL, hierarchical task networks) are used for planning satellite servicing missions and orbital assembly sequences.
- **Grasp planning:** Choosing how to grip an object based on its geometry, the intended placement orientation, and obstacle constraints. A block that needs to be placed vertically might need to be grasped from the side, not the top. Grasp planning becomes even more complex with non-convex, asymmetric modules in space assembly.
- **Structural stability analysis:** Determining whether a partially built structure will stand (or float stably in space). Understanding static equilibrium, center of mass, and support polygons for gravity environments, and rotational dynamics for zero-g environments.
- **Simulation integration:** Combining the physics simulator (MuJoCo) with a planner and controller into a complete autonomous assembly system. This end-to-end integration skill — perception feeds planning, planning feeds control, control drives simulation — is exactly how real space robotics systems are architected.

---

### 6. Dynamic Replanning (moving obstacles)

**How this works toward the end goal:**
Space assembly environments are dynamic — other robots are moving, partially built structures flex and drift, debris may enter the workspace, and communication delays mean ground control can't joystick every movement. Robots must perceive changes and replan autonomously in real time. This project builds reactive autonomy: the ability to commit to a plan but abandon and revise it instantly when conditions change. D* Lite (an incremental replanning algorithm) is particularly relevant because it efficiently updates a previous plan rather than replanning from scratch — critical when computation is limited on space-grade processors.

**Skills and tools — why they matter:**
- **Real-time algorithms:** Planning under time constraints. A robot mid-motion can't pause for 5 seconds to replan — the algorithm must produce a feasible (if not optimal) path within milliseconds. Understanding anytime algorithms (provide the best solution found so far, improve if time allows) is essential for space robots with limited processors.
- **Sensor-driven reactivity:** Tight perception-action loops where vision continuously updates the world model and the planner continuously adapts. This is the feedback architecture that enables autonomy — the robot doesn't just execute a script, it responds to reality.
- **CHOMP / trajectory optimization:** Formulating motion planning as an optimization problem (minimize path length + obstacle cost) rather than a search problem. Optimization-based planners are smoother, more natural, and easier to adapt to additional constraints (joint torque limits, energy consumption) — all relevant for space applications.
- **Safety and constraint handling:** Ensuring the robot never enters a dangerous state even during rapid replanning. Hard constraints (joint limits, collision avoidance, force limits) must be maintained at all times. This discipline is non-negotiable for space systems where a collision could destroy a billion-dollar asset.

---

## Phase 3: Mobile Manipulation

### 7. Mobile Manipulator (base + arm)

**How this works toward the end goal:**
A fixed-base arm can only assemble structures within its reach. Space assembly requires robots that can reposition themselves to access different parts of a growing structure — whether by crawling along truss segments (like GITAI and NASA's concepts), free-flying (like Astrobee on the ISS), or walking (like ESA's walking robot concepts). A mobile manipulator combines navigation ("get to the right location") with manipulation ("do the assembly task"), and the key challenge is coordinating both: the base must position itself so the arm can reach the target, the arm must plan motions relative to a moving base, and the whole system must localize itself relative to the structure.

**Skills and tools — why they matter:**
- **SLAM (Simultaneous Localization and Mapping):** Building a map of the environment while simultaneously tracking the robot's position within it. In space, this means mapping the structure under construction and knowing where the robot is relative to it. SLAM algorithms (gmapping, cartographer, ORB-SLAM) are fundamental to autonomous navigation everywhere from warehouses to Mars.
- **ROS2 Nav2 stack:** The standard autonomous navigation framework. Understanding costmaps, global/local planners, behavior trees, and recovery behaviors teaches the architecture of production-grade autonomous navigation. The same architectural patterns apply to space navigation software.
- **LiDAR / depth camera processing:** Converting raw sensor data (point clouds, depth images) into actionable information (obstacle locations, surface normals, object recognition). Point cloud processing (PCL library, Open3D) is used extensively in space robotics for structure inspection and docking.
- **Whole-body coordination:** Planning motions for the base and arm simultaneously rather than sequentially ("drive, then reach"). This is an optimization problem — find the base pose that maximizes arm manipulability at the target. The same problem exists for free-flying space robots that must orient their thrusters while positioning their arm.
- **Localization in changing environments:** The environment changes as you assemble — your map from 10 minutes ago doesn't include the truss segment you just placed. Handling dynamic maps and re-localization is a specific challenge of construction robotics, both terrestrial and in space.

---

### 8. Pick-and-Place in Unstructured Environments

**How this works toward the end goal:**
Lab conditions are perfect — objects on clean tables with consistent lighting. Space is not. Components may be tethered and drifting, reflecting harsh sunlight from one angle and in deep shadow from another, or partially obscured behind existing structure. This project pushes perception and manipulation to handle uncertainty, clutter, and partial information. A self-assembling space robot must reliably grasp modules that aren't perfectly positioned, recover from failed grasps without losing the component (which would become debris), and adapt its strategy when things don't go as planned.

**Skills and tools — why they matter:**
- **Depth cameras and 3D perception (Intel RealSense, stereo vision):** Moving beyond 2D color images to 3D point clouds that capture object geometry. Depth data enables 6-DOF pose estimation (where is the object AND how is it oriented), which is essential for grasping non-trivial objects. Space robots use stereo cameras and structured light for 3D perception.
- **Point cloud processing:** Filtering noise, segmenting surfaces, estimating normals, aligning point clouds (ICP algorithm). These operations are the 3D equivalent of the 2D image processing in Phase 1 and are the standard perception pipeline for space robotics.
- **Grasp pose detection:** Analytically computing or learning where and how to grasp an object given its shape. Approaches range from geometric analysis (find parallel surfaces for a parallel gripper) to deep learning (GraspNet, Contact-GraspNet). Understanding both analytical and learned approaches provides flexibility.
- **Failure detection and recovery:** Monitoring grasp success via force feedback, slip detection, or visual verification. Implementing retry strategies, alternative grasp poses, and graceful degradation. In space, a dropped component becomes orbital debris — failure recovery is mission-critical.
- **Robustness to perception uncertainty:** Working with noisy, incomplete, or ambiguous sensor data. Bayesian approaches (maintaining probability distributions over object poses), data fusion (combining multiple sensor modalities), and active perception (moving the camera to get a better view) all build the reliability needed for space operations.

---

## Phase 4: Multi-Robot Coordination

### 9. Two-Robot Cooperative Assembly

**How this works toward the end goal:**
Self-assembling space structures can't be built by one robot alone — the task requires coordinated teams. Two robots assembling together introduces the fundamental challenges of multi-robot systems: shared workspace collision avoidance (don't hit each other), task synchronization (one holds the beam while the other bolts it), communication protocols (how do they share state?), and deadlock prevention (both robots waiting for the other to move first). Solving these for two robots provides the framework that scales to many robots. Every proposed large-scale space assembly concept (from NASA's in-space manufacturing roadmap to commercial ventures like Orbital Reef) requires multi-robot coordination.

**Skills and tools — why they matter:**
- **Multi-agent motion planning:** Planning collision-free paths for multiple robots simultaneously. The joint configuration space is the product of individual configuration spaces (two 6-DOF arms = 12D planning problem), making it computationally challenging. Approaches include prioritized planning (plan for one robot, treat it as a moving obstacle for the other), coupled planning (plan jointly), and reactive methods (velocity obstacles). These directly apply to multi-robot space assembly.
- **Communication protocols:** Designing message formats and communication patterns between robots. What information is shared (joint states? planned paths? detected objects?)? How often? What happens when a message is lost? In space, communication faces latency and bandwidth constraints — efficient protocols matter.
- **Synchronization and handoffs:** Coordinating tightly coupled tasks where timing matters — one robot must grip before the other releases, both must lift simultaneously for heavy objects. Implementing state machines with handshake protocols teaches the coordination patterns used in real multi-robot systems.
- **Distributed vs. centralized architectures:** A central planner knows everything but is a single point of failure. Fully distributed systems are robust but harder to coordinate. Understanding this trade-off and implementing both approaches provides insight into the architecture decisions facing space assembly system designers.
- **Shared world models:** Multiple robots must agree on the state of the world — where are the components, what's been assembled, what's each robot doing? Maintaining consistency across distributed world models is a computer science challenge (consensus algorithms, conflict resolution) with direct applications in multi-robot space systems.

---

### 10. Distributed Task Allocation

**How this works toward the end goal:**
When you have 10 modules to place and 4 robots available, which robot places which module? Optimal task allocation minimizes total assembly time and energy consumption. In space, this optimization is critical — fewer robot-hours means less propellant, less wear on mechanisms, and faster assembly of time-critical structures (like a habitat that must be pressurized before crew arrives). As robot teams grow larger, centralized planning becomes computationally infeasible, so distributed algorithms where robots locally negotiate task assignments become necessary. This project builds the decision-making layer that sits above individual robot control.

**Skills and tools — why they matter:**
- **Auction-based algorithms:** Robots "bid" on tasks based on their cost to complete them (distance to travel, energy required, current workload). The task goes to the lowest bidder. Understanding auction mechanisms (first-price, second-price, combinatorial auctions) provides a practical framework for distributed resource allocation that scales well and handles dynamic task arrivals.
- **Optimization (linear programming, integer programming):** Formulating task allocation as a mathematical optimization problem. The Hungarian algorithm for optimal assignment, vehicle routing problems, and scheduling theory all apply. These optimization techniques are used directly in mission planning for space operations.
- **Game theory and mechanism design:** Understanding strategic behavior when multiple autonomous agents interact. How do you design allocation rules that incentivize truthful bidding? How do you prevent deadlocks or starvation (one robot gets all the tasks)? These concepts are fundamental to designing robust multi-agent systems.
- **Dynamic reallocation:** Real systems face disruptions — a robot breaks down, a new task appears, priorities change. Implementing reallocation without restarting from scratch (incremental repair of existing assignments) builds the adaptability that space systems need to handle anomalies.
- **Simulation at scale:** Testing with 4, 8, 16, or more simulated robots to understand how algorithms scale. Performance profiling, bottleneck identification, and scalability analysis are essential skills for designing systems that work at the scale required for large space structures.

---

## Phase 5: Modular & Reconfigurable Robotics

### 11. Modular Docking Robot Units

**How this works toward the end goal:**
Self-assembling robots ARE modular robots. Each module is an independent robot that can connect to other modules to form larger structures or reconfigure into different shapes. The docking mechanism is the critical hardware interface — it must be reliable (dock thousands of times without failure), provide structural rigidity (the assembled structure must bear loads), transfer power and data between modules, and work in the extreme thermal and vacuum conditions of space. This project moves from software-only to hardware design, bridging the gap between simulation and physical systems. Every proposed self-assembling space system (SMORES, M-Blocks, SUPERBALL, NASA's concepts) centers on modular docking.

**Skills and tools — why they matter:**
- **Mechanical design and CAD:** Designing the physical docking mechanism — magnetic coupling for easy alignment, mechanical latches for structural strength, electrical connectors for power/data transfer. Understanding tolerances, material selection, and mechanism kinematics. CAD tools (SolidWorks, Fusion 360) and rapid prototyping (3D printing, laser cutting) enable iterative hardware development.
- **Embedded systems design:** Each module needs its own processor, sensors (to detect alignment for docking), actuators (to engage/disengage the latch), and communication interface. Designing compact, power-efficient embedded systems with real-time constraints teaches the hardware-software co-design that space robotics demands.
- **Communication bus design:** Modules must communicate through their docking interfaces — sharing state, coordinating actions, propagating commands. Designing a robust communication protocol that works across physical connections (I2C, CAN bus, or custom) and handles hot-plugging (modules connecting/disconnecting while powered) is a non-trivial embedded systems challenge.
- **Module standardization:** Defining the interface specification so that any module can connect to any other module in any valid orientation. This requires thinking about symmetry groups, connection compatibility, and interface versioning — the same design challenges facing space module interface standards (like the iSSI Standard Interconnect).
- **Testing and reliability:** Docking mechanisms must work every time. Designing test rigs, characterizing failure modes, and implementing fault detection builds the reliability engineering mindset essential for space hardware.

---

### 12. Self-Reconfiguration Algorithms

**How this works toward the end goal:**
This is the algorithmic heart of self-assembling robotics. Given a connected set of modules in shape A, compute a sequence of moves (disconnect module X, move it, reconnect at position Y) that transforms the assembly into shape B, while ensuring the structure remains connected and stable at every intermediate step. This is computationally hard (the state space grows exponentially with module count) and physically constrained (modules can only move if they can physically detach and there's a clear path to the new position). These algorithms are the "brain" that would enable a swarm of space modules to autonomously build a space station, reshape into an antenna, or repair a damaged structure by redistributing modules.

**Skills and tools — why they matter:**
- **Graph theory:** Representing robot configurations as graphs (nodes = modules, edges = connections) and reconfiguration as graph transformation. Understanding graph connectivity, graph isomorphism (are two configurations the same shape?), and minimum spanning trees provides the mathematical language for reconfiguration.
- **Planning in combinatorial spaces:** The state space of possible configurations is enormous. Techniques from AI planning — heuristic search, sampling-based planning, hierarchical decomposition (reconfigure large subassemblies, then assemble subassemblies) — are essential for finding solutions in reasonable time.
- **Cellular automata and local rules:** An alternative to centralized planning where each module follows simple local rules (look at your neighbors, decide whether to move) that produce global reconfiguration behavior. Understanding emergence — complex global behavior from simple local rules — is fundamental to scalable self-assembly.
- **Locomotion through reconfiguration:** Modular robots can move by rearranging their own modules (like a caterpillar). Understanding gaits, stability during locomotion, and efficient locomotion strategies connects reconfiguration to mobility.
- **Formal verification:** Proving that a reconfiguration algorithm will always succeed (completeness) and never leave the structure disconnected (safety). Mathematical proof techniques and model checking build the rigor needed for space-critical algorithms where failure isn't an option.

---

## Phase 6: Space Environment

### 13. Microgravity Sim + Assembly

**How this works toward the end goal:**
This project directly simulates the target environment: assembling structures in space. Removing gravity changes everything — objects don't fall, they drift. Pushing on a module pushes you away (Newton's third law). Structures don't need foundations but also don't stay in place. Every control algorithm, every assembly sequence, every grasp strategy must be reconsidered for zero-g. This is where all previous phases converge: the arm control from Phase 1, the planning from Phase 2, the perception from Phase 3, the multi-robot coordination from Phase 4, and the modular systems from Phase 5, all adapted for the unique physics of space. The results from this phase become thesis content and publishable research.

**Skills and tools — why they matter:**
- **Orbital mechanics and microgravity physics:** Understanding the forces acting on objects in orbit — gravity gradients, atmospheric drag (in LEO), solar radiation pressure, electromagnetic interactions. These perturbation forces affect assembly precision and must be modeled in simulation and compensated by controllers.
- **Free-floating dynamics:** When the base isn't fixed, the robot and the object form a coupled dynamic system. Grasping and moving an object changes the robot's own trajectory. Understanding coupled dynamics, momentum conservation, and reaction control is essential for space manipulation. Dual quaternion representations, spatial algebra (Featherstone's algorithm), and multibody dynamics are the mathematical tools.
- **Thruster and reaction wheel control:** Space robots use thrusters or reaction wheels for attitude control and station-keeping. Understanding actuator models (thrust curves, fuel consumption, momentum saturation), control allocation (distributing desired forces/torques across multiple thrusters), and fuel-optimal trajectories is specific to space robotics.
- **Custom physics simulation:** Modifying MuJoCo or writing custom simulation components to accurately model space-specific phenomena (vacuum, thermal effects on mechanisms, flexible structure dynamics). This requires deep understanding of numerical methods and physics modeling.
- **Space systems engineering context:** Understanding the broader context — mission design, launch constraints (mass, volume, vibration), power budgets, thermal management, radiation effects — that shapes what's feasible. A brilliant algorithm that requires too much computation for a radiation-hardened processor is impractical. This systems-level thinking distinguishes space robotics from lab robotics.

---

### 14. Fault Tolerance & Self-Repair

**How this works toward the end goal:**
Space is unforgiving — there's no technician to reboot a crashed module, replace a burned-out motor, or untangle a communication failure. Self-assembling space robots must detect their own failures and recover autonomously. This is what makes self-assembling robotics transformative for space: not just the ability to build, but the ability to maintain and repair without human intervention. A structure that can detect a damaged module, dispatch a robot to remove it, reconfigure the remaining modules to maintain function, and eventually replace the damaged module with a spare from storage — this is the full vision of self-sustaining space infrastructure.

**Skills and tools — why they matter:**
- **Health monitoring and diagnostics:** Implementing continuous self-checks — motor current draw (indicates mechanical binding), sensor consistency (cross-check redundant sensors), communication latency (indicates network problems), thermal monitoring (overheating warnings). Understanding statistical anomaly detection (is this sensor reading normal?) builds the ability to catch problems early.
- **Fault detection, isolation, and recovery (FDIR):** The formal framework used in spacecraft design. Detection: something is wrong. Isolation: identify what specifically failed. Recovery: take corrective action. Implementing FDIR logic (often as hierarchical state machines or behavior trees) teaches the systematic approach to reliability used in all space systems.
- **Redundancy design:** Hardware redundancy (backup sensors, redundant communication paths) and functional redundancy (if the left arm fails, the right arm can do its tasks, just slower). Understanding redundancy trade-offs (more reliability vs. more mass/cost/complexity) is a core space systems engineering skill.
- **Graceful degradation:** Designing systems that lose capability gradually rather than failing completely. A 10-module assembly robot that loses 2 modules should still function at reduced capacity, not shut down entirely. Implementing capability assessment ("what can I still do?") and mission replanning ("what's the best I can achieve now?") teaches the resilient system design that space demands.
- **Self-repair through reconfiguration:** Connecting back to Phase 5 — using self-reconfiguration algorithms to physically replace a failed module. The system detects a dead module in the structure, dispatches a robot to remove it, retrieves a spare, and installs it. This is the ultimate expression of autonomous maintenance and a compelling thesis contribution.
- **Verification and validation:** Testing fault tolerance requires injecting faults systematically — killing processes, disabling motors, corrupting sensor data, severing communication links — and verifying that the system recovers correctly every time. Designing comprehensive fault injection test suites builds the V&V skills that are mandatory for any space-rated system.
