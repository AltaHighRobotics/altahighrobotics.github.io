---
title: Robotics Notes 
parent: Robotics
---
___

### **Overview of Swerve Drive Base Code**

# Tech Stack

* **Language**: **Java** (officially supported by WPILib)

* **Libraries/Dependencies**:

  * **WPILib** (core robot control library)  
    * Provides classes like `TimedRobot`, `Command`, `SubsystemBase`, `PIDController`, and `SwerveDriveKinematics`.

    * Manages lifecycle methods (`robotInit()`, `autonomousInit()`, `teleopPeriodic()`).

    * Includes math tools (e.g. `Rotation2d`, `ChassisSpeeds`) for drivetrain control and motion planning.

    * Enables command-based programming for modular, testable code.

  * **REV Robotics SPARK MAX API**  
    * Allows you to configure motor mode (brake/coast), PID loops, encoders, and follow modes.

    * Provides access to motor state and control (e.g., set power, set velocity, get encoder position).

    * Supports both brushed and brushless motors.

  * **PhotonVision / Limelight** (optional for vision)  
    **Use:** Real-time vision processing for targeting and localization.  
    **What they do:**  
* Identify AprilTags, retroreflective tape, or other targets.

* Return 3D pose estimates of the robot using camera-to-target geometry.

* Can be fused with odometry to improve field positioning accuracy.

  * **CTRE Phoenix 6 (optional if you use any CTRE sensors)**

  * **NavX or Pigeon (gyro support)**  
    **Use:** Inertial Measurement Unit (IMU) for measuring rotation and orientation.  
     **What it does:**  
* Tracks the robot’s orientation on the field using a gyro and accelerometers.

* Used for field-relative drive and for pose estimation in autonomous routines.

* Easily integrates via SPI or I2C.

  **Example in your code:**

* `AHRS gyro = new AHRS(SPI.Port.kMXP);` in `SwerveDrivetrain.java`.


  * **PathPlanner** or **WPILib Trajectory API** (for auto path following)  
    **Use:** Path generation and trajectory following in autonomous mode.  
     **What it does:**  
* Lets you design complex autonomous paths with turns and velocities.

* Can export these to JSON files read by your code during a match.

* Supports swerve-specific autonomous following with rotation goals.

* Works well with WPILib's trajectory tracking and command-based structure.

  **Typical usage (you’ll add later):**

* Load JSON paths.

* Feed into trajectory-following command like `PPSwerveControllerCommand`.


#### **Main Code Subsystems**

1. **SwerveModule**

   * Controls a single wheel's drive & angle motors.

   * Handles PID control & encoder readings.

2. **SwerveDrivetrain**

   * Coordinates all 4 modules.

   * Handles kinematics, odometry, gyro integration.

3. **RobotContainer**

   * Binds commands to joysticks/controllers.

   * Sets up button mappings & default commands.

4. **Commands**

   * DriveCommand: reads controller input & tells drivetrain where to go.

   * AutoPathFollowCommand: runs autonomous paths.

5. **Main Robot Class**

   * Initializes all subsystems.

   * Handles autonomous/teleop/disabled modes.

---

# Key Libraries and Setup

`// build.gradle`  
`dependencies {`  
    `implementation wpi.java.deps.wpilib()`  
    `implementation 'com.revrobotics.frc:SparkMax-java:1.5.6'`  
    `implementation 'com.pathplanner:PathplannerLib:2024.1.0'`  
    `implementation 'com.kauailabs.navx.frc:navx-java:2023.1.1'`  
`}`

# SPARK MAX Major Functions

### **1\. Basic Motor Control**

`motor.set(double speed);`

* Range: `-1.0` to `1.0` (percent output)

* Used for open-loop control (manual power control)

---

### **2\. Motor Configuration**

`motor.setInverted(true);  // Reverse direction`  
`motor.setIdleMode(CANSparkMax.IdleMode.kBrake);  // or kCoast`  
`motor.restoreFactoryDefaults();  // Reset all settings`

* Controls direction and behavior when no signal is sent.  
  ![][image1]

---

### **3\. Encoder Access**

`RelativeEncoder encoder = motor.getEncoder();`  
`double position = encoder.getPosition(); // rotations`  
`double velocity = encoder.getVelocity(); // RPM`  
`encoder.setPosition(0); // reset encoder`

* Lets you track how far and how fast the motor has moved.

---

### **4\. Current Limiting**

`motor.setSmartCurrentLimit(40);  // max amps`

* Protects your robot’s power system and motors.

* Prevents brownouts or blown breakers.

  #### **Two Ways to Set Current Limits**

  ##### 1\. In Code (Java)

  `driveMotor.setSmartCurrentLimit(40);  // 40 amps max`  
  Set during robot code execution.  **Applies at runtime** every time the robot boots up.  **Temporary** unless explicitly saved with `burnFlash()`.

  ##### 2\. Using REV Hardware Client (USB-C)

* Plug controller into your laptop via USB-C.  Open the **REV Hardware Client**.  Navigate to “Current Limits” tab.  Set values and **click "Burn Flash"** to save.  **Persistent** even if you change robot code later.

  ---

  #### Key Differences in Current Limit Methods

![][image2]

#### **Danger of USB-only Configuration**

If you configure limits with the Hardware Client but **don’t** set them (or accidentally override them) in code — they could be lost during operation.

That’s why this is best practice:

---

#### Best Practice for FRC

* `motor.setSmartCurrentLimit(40);`  
* `motor.burnFlash();  // Makes it permanent on controller`  
    
  This way:  
* You define it in code (easy to see and version control)

* It **sticks after a reboot**, just like if you used USB

---

### **5\. Ramp Rate**

`motor.setOpenLoopRampRate(0.5); // seconds to full throttle`

* Smooths out acceleration to **reduce stress on gearboxes.**

* Prevents sudden jerks or tipping.

---

### **6\. Internal PID Control**

`SparkMaxPIDController pid = motor.getPIDController();`  
`pid.setP(0.1);`  
`pid.setI(0.0);`  
`pid.setD(0.0);`

`pid.setReference(velocityTarget, ControlType.kVelocity);  // or kPosition`

* SPARK MAX can close the loop *on its own hardware*, reducing CPU load on the RoboRIO.

* You can use this for velocity or position control (used for swerve turn motors\!).

---

### **7\. Follower Mode**

`otherMotor.follow(motor);`

* Makes one motor copy the output of another (useful for drivetrains with 2+ motors per side).

---

### **8\. Burn to Flash**

`motor.burnFlash();`

* Saves configuration permanently to the motor controller.

* Must be called if you want settings like current limits to persist after reboot.

---


#### Example Use Case: Drive Motor

`CANSparkMax driveMotor = new CANSparkMax(1, MotorType.kBrushless);`  
`driveMotor.setSmartCurrentLimit(40);`  
`driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);`

`RelativeEncoder encoder = driveMotor.getEncoder();`  
`double velocity = encoder.getVelocity();`

---

#### Summary Table

| Function | Description | Used In |
| ----- | ----- | ----- |
| `set(double)` | Set motor speed | Open-loop drive |
| `getEncoder()` | Access Neo internal encoder | Feedback loop |
| `getPIDController()` | Use internal PID for smart control | Swerve turning |
| `setSmartCurrentLimit()` | Prevent breaker trips / brownouts | Protection |
| `setIdleMode()` | Brake or coast at 0% output | Drivetrain |
| `burnFlash()` | Save settings to motor | Setup |

### Setting PID Parameters

#### `setP(1.0)` – Proportional Gain

* Reacts to **current error**: how far the wheel is from its target angle

* The bigger the error, the stronger the motor pushes

* A value of `1.0` is fairly aggressive (adjust as needed)

🧠 Think: *“Push hard if the wheel is far off.”*

---

#### `setI(0.0)` – Integral Gain

* Reacts to **accumulated error over time**

* Useful to eliminate small steady-state errors

* Can cause instability if set too high

🧠 Think: *“If I’ve been off target for a while, push harder to correct.”*

Often left at `0.0` in FRC unless fine tuning.

---

#### `setD(0.0)` – Derivative Gain

* Reacts to **rate of change of error**

* Helps dampen overshoot and oscillation

* Adds braking effect if the wheel is moving too fast

🧠 Think: *“If I’m changing too fast, slow down.”*

Try small values like `0.01` if you see oscillation.

# WPILib Overview

**WPILib** is the core software library provided by FIRST and WPI for writing code to control FRC robots. It’s designed to abstract away low-level hardware details while giving teams powerful tools for sensors, motor control, input devices, and autonomous programming.

---

### **Core Features & What They Do**

| Feature Area | Purpose | Example Classes/Interfaces |
| ----- | ----- | ----- |
| **Robot lifecycle** | Handles robot modes and system initialization | `TimedRobot`, `RobotBase` |
| **Command-based** | Modern architecture to separate logic from control flow | `Command`, `CommandBase`, `SubsystemBase` |
| **Motor control** | Interfaces with motor controllers and sensors | `PWMSparkMax`, `Talon`, `MotorControllerGroup` |
| **Math \+ kinematics** | Tools for odometry, trajectory, geometry math | `Rotation2d`, `Pose2d`, `SwerveDriveKinematics` |
| **Sensors & I/O** | Read values from hardware like encoders, gyros, switches | `DigitalInput`, `Encoder`, `AnalogGyro`, `AHRS` |
| **Autonomous paths** | Plan and follow motion profiles | `Trajectory`, `RamseteCommand`, `Timer` |
| **Field simulation** | Simulate sensors and robot movement in software | `Field2d`, `SimDevice`, `SimulationGUI` |

---

### **Robot Lifecycle**

WPILib provides a class called `TimedRobot` that defines the robot’s state machine:

`public class Robot extends TimedRobot {`  
    `public void robotInit() { }        // Runs once at boot`  
    `public void teleopPeriodic() { }   // Runs repeatedly during teleop`  
    `public void autonomousInit() { }   // Called once at auto start`  
`}`

Each method corresponds to a **match phase**, making it easy to separate autonomous, teleop, and test behavior.

---

### **Command-Based Programming Model**

The command-based model is a structure that breaks your robot code into:

* **Subsystems**: hardware units like drivetrain, shooter, arm

* **Commands**: actions the robot performs (e.g., drive, aim, shoot)

* **RobotContainer**: central place to bind buttons to commands and instantiate subsystems

This architecture makes the robot more modular, easier to debug, and supports simultaneous tasks.

---

### **WPILib Math & Kinematics**

WPILib includes tools for advanced drivetrain control:

* `SwerveDriveKinematics`: converts desired chassis speeds to module states

* `ChassisSpeeds`, `SwerveModuleState`: represent linear and angular motion

* `Pose2d`, `Rotation2d`: for field-relative positioning

* `Trajectory`, `RamseteCommand`: path-following for autonomous

These are crucial for swerve, mecanum, and tank drive odometry and auto modes.

---

### **Joysticks & Controllers**

WPILib makes it easy to use Xbox/Logitech controllers:

`XboxController controller = new XboxController(0);`  
`double x = controller.getLeftX();`

Supports all axes, buttons, and rumble feedback.

---

### **Sensors & Inputs**

Supports a wide range of devices:

* `Encoder` – wheel or arm position/speed

* `DigitalInput` – limit switches

* `AnalogInput` – potentiometers

* `Gyro` or `AHRS` (NavX) – rotation/orientation

Also integrates vision systems and LIDAR through custom classes or vendor libraries.

---

### **Simulation Tools**

You can run robot code **without hardware** using:

* `Field2d` – shows robot position on a virtual field

* `SimDevice` – inject values into fake encoders, motors, sensors

* WPILib GUI tools and **Glass** dashboard for live tuning

---

### **WPILib in Practice (Your Swerve Project)**

| Component | WPILib Class Used | What It Does |
| ----- | ----- | ----- |
| Joystick Input | `XboxController` | Gets movement input from driver |
| Drivebase | `SwerveDriveKinematics` | Translates speed to module states |
| Pose Estimation | `Rotation2d`, `ChassisSpeeds` | Tracks heading and velocity |
| Command Loop | `CommandScheduler` | Runs default commands and bindings |
| PID Math | `PIDController` | Fine control over motion |

## 

# NavX (Kauai Labs)

### **Advantages**

* Plug-and-play with SPI or USB

* Easy to mount on RoboRIO or chassis

* Great WPILib support

* Works directly with `AHRS` class

### **Connection Types**

* **SPI (recommended)** – fast and reliable

* **I2C** – slower, simpler

* **USB** – fast but takes RoboRIO port

### **Code Example (NavX)**

`import com.kauailabs.navx.frc.AHRS;`  
`import edu.wpi.first.wpilibj.SPI;`

`AHRS gyro = new AHRS(SPI.Port.kMXP);`  
`double angle = gyro.getAngle();     //`   
`double yaw = gyro.getYaw();         // -180 to 180`  
`double pitch = gyro.getPitch();`  
`double roll = gyro.getRoll();`

### **Tip: Reset Orientation**

`gyro.reset();  // Sets angle to 0`

---

# Pigeon (CTRE)

### **Advantages**

* Native CAN bus communication

* Excellent integration with **Phoenix** (TalonSRX/TalonFX)

* Can be **mounted on a motor controller** (Pigeon 1 on TalonSRX)

### **Models**

* **Pigeon 1.0** – connects to Talon SRX or CAN directly

* **Pigeon 2.0** – stand-alone, fully CAN connected, improved precision

### **Code Example (Pigeon2 – Phoenix 6\)**

`import com.ctre.phoenix6.hardware.Pigeon2;`

`Pigeon2 pigeon = new Pigeon2(0); // CAN ID`  
`Rotation2d heading = Rotation2d.fromDegrees(pigeon.getYaw().getValue());`

For Pigeon 1.0 on Phoenix 5:

`PigeonIMU pigeon = new PigeonIMU(0);`  
`double[] ypr = new double[3];`  
`pigeon.getYawPitchRoll(ypr);`  
`double yaw = ypr[0];`

---

## **NavX vs. Pigeon — Which Should We Use?**

| Feature | NavX | Pigeon |
| ----- | ----- | ----- |
| **Setup** | Easier for rookies | Better with CAN-based drivetrains |
| **Data access** | `AHRS` class via WPILib | CTRE Phoenix API (5 or 6\) |
| **Mounting** | Easy on RoboRIO or chassis | Best on motor controller or CAN |
| **Precision** | Comparable in FRC context | Slightly faster with Pigeon 2.0 |
| **Drift** | Both are stable over match time | Both use fusion (gyro \+ accel) |

---

**When to Use IMUs**

* Field-relative drive (swerve/mecanum)

* Rotate-to-angle commands

* Odometry & pose estimation

* Autonomous path following

# PhotonVision

### **Key Features**

* Open-source and customizable

* Runs on Raspberry Pi, Pi Pico, Jetson, etc.

* AprilTag detection with pose estimation built in

* NetworkTables-based

* Supports multi-camera systems

* Integrates with WPILib through `PhotonCamera`

### **Example Usage (Java)**

`PhotonCamera camera = new PhotonCamera("photonvision");`

`// Get latest result`  
`PhotonPipelineResult result = camera.getLatestResult();`

`if (result.hasTargets()) {`  
    `PhotonTrackedTarget target = result.getBestTarget();`

    `double yaw = target.getYaw();      // Horizontal angle to target`  
    `double pitch = target.getPitch();  // Vertical angle`  
    `double area = target.getArea();    // Target area on screen`  
`}`

### **AprilTag Pose Estimation**

`AprilTagFieldLayout layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();`  
`PhotonPoseEstimator estimator = new PhotonPoseEstimator(`  
    `layout,`  
    `PoseStrategy.AVERAGE_BEST_TARGETS,`  
    `camera,`  
    `cameraToRobotTransform`  
`);`

`Optional<EstimatedRobotPose> pose = estimator.update();`

---

# Limelight

### **Key Features**

* Commercial plug-and-play vision camera

* Web dashboard for tuning and pipelines

* AprilTags and reflective tape tracking

* Fast, low-latency image processing

* Simple integration via NetworkTables

### **Example Usage (Java)**

`NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");`

`double tx = limelightTable.getEntry("tx").getDouble(0.0); // Horizontal offset`  
`double ty = limelightTable.getEntry("ty").getDouble(0.0); // Vertical offset`  
`double ta = limelightTable.getEntry("ta").getDouble(0.0); // Target area`

### **Limelight AprilTag Pose Estimation**

`double[] botPose = limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);`  
`Pose2d pose = new Pose2d(botPose[0], botPose[1], Rotation2d.fromDegrees(botPose[5]));`

---

## **Common Use Cases in FRC**

| Use Case | PhotonVision | Limelight |
| ----- | ----- | ----- |
| Auto-alignment to goals | Supported | Supported |
| AprilTag pose estimation | Built-in | Limelight 3 only |
| Retroreflective tape tracking | Supported | Supported |
| Game piece detection (color) | Customizable | Via pipelines |
| Multi-camera support | Built-in | Limited |
| Ease of setup | Moderate | Very easy |

---

## **Field-Relative Integration**

Both libraries can be used to **update your robot’s pose** based on visual input, improving accuracy during autonomous and aiding odometry corrections.

---

## **Library Integration Summary**

| Vision System | Java Integration | Key Classes or Tools | Primary Usage |
| ----- | ----- | ----- | ----- |
| PhotonVision | WPILib \+ PhotonLib | `PhotonCamera`, `PhotonPoseEstimator` | Vision pose \+ aiming |
| Limelight | NetworkTables API | `NetworkTable.getEntry()` | Simple aiming \+ pose |

# Extras:

## RoboRIO images
Download 2025 roborio image: [external link](https://pizza2d1.duckdns.org/files/WorkingRoboRIO.img.gz) [unblocked external link](http:136.60.227.41/files/WorkingRoboRIO.img.gz)
##### Linux:
###### Create a compressed image file with a working image file

```bash
dd if=/dev/sdb | gzip > backup.img.gz
```

###### Flash a drive with compressed image file

```bash
cat backup.img.gz | gunzip | dd of=/dev/sdb
```


Example code for our purposes: 
###### This will echo the contents of the RoboRio image file, decompress it with gunzip, and then overwrite existing data on the /dev/mmcblk0p1 storage device, status and bs parameters add verbosity and speed
```bash
sudo cat WorkingRoboRIO.img.gz status=progress bs=32M| gunzip | dd of=/dev/mmcblk0p1 status=progress bs=32M
# Make sure you are outputting to the right drive and not your own, that will brick your laptop
```


## Virtual Environment
Get into a virtual environment:
##### Windows:
Run in powershell:
```batch
Set-ExecutionPolicy -ExecutionPolicy Unrestricted -Scope CurrentUser
```

Run in repository destination:
```
venv env

# Or if your python is weird:
python3 -m venv env
```
## NOT FINISHED

##### Linux:
Make sure that both `python3.12-venv` and `python3-pip` are installed beforehand

In the current directory
```bash
# Create virtual environment
python3 -m venv .env

# Enter virtual environment
source .env/bin/activate
```

Once you are in the virtual environment you are able to download any packages that you want and run the modules without any issue, BUT once you leave the environment (which can be seen in terminal as no longer having `(.env)` at the terminal entry line)

To get back into the virtual environment, you just run the same source command:
```bash
source .env/bin/activate
```
Which you may have to do each time you want to use that specific environment
