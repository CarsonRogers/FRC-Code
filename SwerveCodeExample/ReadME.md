# Creating Swerve Drive
## A swerve drive subsystem with code from FRC 461 and FRC 3175
<details>
    <summary>Content</summary>
    <ol>
        <li><a href="#works">How Swerve Works</a></li>
        <li><a href="#module">Swerve Module</a></li>
        <li><a href="#swerve">Swerve Subsystem</a></li>
        <li><a href="#command">Swerve Drive Command</a></li>
        <li><a href="#init">Adding and Initializing</a></li>
        <li><a href="#problem">Common Problems with Swerve</a></li>
        <li><a href="#words">Key Words and Names</a></li>
    </ol>
</details>

<div id="works"></div>

# How Swerve Works

Swerve drive is a complex drivetrain that allows for more complex movements. The mechanical side of swerve consists of four swerve modules that each control a wheel for a total of four independantly controlled wheels which allows the robot to move in any direction and rotation at any time. 

From a programming perspective, swerve drive follows a few key steps
* 1. Get joystick inputs and apply filters to the inputs
* 2. Convert the inputs to a desired translation and rotation
* 3. Convert the translation and rotation to the desired speed of the robot chassis
* 4. Convert the chassis speeds to a speed and rotation for each swerve module
* 5. Apply the speeds and rotations for all swerve modules

From these steps, swerve drive can be broken down into its parts to be created.

<div id="module"></div>

# Swerve Module

The first part of creating a swerve drive system is the swerve module. Each swerve module consist of a few parts and is responsible for driving and turning to the desired state of the module.

The first step is declaring everything needed for a swerve module. Each swerve module has:
* A module number: The number of the module in the array of modules
* A name: The name of the module like Front Right or Back Left
* The encoder/angle offset: The offset of the module's encoder to zero the module so that front facing is 0
* The desired State: The desired state (speed and rotation) of the module
* Angle Motor: The angle motor that turns the module
* Drive Motor: The drive motor that drives the module
* Angle Encoder: The absolute encoder that knows where the wheel faces at all times, and retains its value even when powered off. This is often a Thriftybot Encoder or a CANcoder
* Integrated Angle Encoder: The relative encoder that knows where the wheel faces when the robot is powered
* Drive Encoder: The relative encoder that knows the position and rotations of the wheel
* Drive Controller: The PID Controller responsible for controlling the drive motor
* Angle Controller: The PID Controller responsible for controlling the angle motor

Once everything is declared, within the SwerveModule Contructor, everything will need initialized

This is where the motors and encoders are configured based off of the motorIDs and encoderIDs. These IDs can be declared and initialized in the Constants file and the called in the SwerveModule file. 

The Constants file is useful as the user only has to go to one file to change any number or id for the robot instead of looking through code to find where to change the id.

Next, methods are called to configure the motors and encoders to set all their defaults like idle modes and conversion factors. The methods also 

The conversion factors are used for the encoders to convert motor rotations to position or speed.

A motor has two different idle modes, coast and brake. The difference is brake activily stops the motor when is not powered, while coast lets it keep spinning until it stops. 

After all the encoders and motors are configured, their are two main methods used for swerve modules. These are the setSpeed() and setAngle() methods.

```
    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
    if (isOpenLoop){
      double percentOutput = desiredState.speedMetersPerSecond / DriveConstants.MaxSpeed;
      driveMotor.set(percentOutput);
    } else {
      driveController.setReference(
        desiredState.speedMetersPerSecond,
        ControlType.kVelocity,
        0,
        drivFeedForward.calculate(desiredState.speedMetersPerSecond));
    }
  }
```

The setSpeed() method takes a desired state and if its open loop. Based on if the robot is open loop or not, the drive motor outputs a value relative to the max speed of the robot.

Open loop control means there is no position feedback of the wheel when moving. Closed loop control means that there is position information fed back to the feedforward controller and that is used to help determine the output speed

```
    private void setAngle(SwerveModuleState desiredState){
    Rotation2d angle = 
      (Math.abs(desiredState.speedMetersPerSecond) <= (DriveConstants.MaxSpeed * 0.01))
      ? lastAngle
      : desiredState
        .angle;

    angleController.setReference(angle.getDegrees(), ControlType.kPosition);
    lastAngle = angle;
  }  
```

The second method is the setAngle() method. This method takes in the desired state and rotates the wheel to the correct heading. A PID Controller is used to set the rotation of the wheel by calculated an output based off of the different between the current and desired rotation. As the difference gets smaller, the output lessens and vice versa if the difference gets larger, the output becomes greater.

If the speed is less than 1% of the max speed, then the module will keep the last angle instead of turning to the new angle. This helps prevent jittering of the swerve module.

```
public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);
    this.desiredState = desiredState;
    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }
```

The next part is combining both methods into one method to set the desired state of the module. This will take the inputted desired state and pass it to the setAngle() and setSpeed() methods.

One important part is optimizing the desired state. Optimizing is where if the wheel needs to turn to a certian degrees, it has to rotate all the way around to reach the angle and then drive forward. Instead the wheel can rotate a shorter angle if possible and rotate backwards to still move in the correct direction, and it takes less time to rotate the wheel. With optimizing, a wheel at most has to rotate 90 degrees and it can reach any angle. 

This example uses a modified version of WPILIb's optimize method to include REV's and CTRE's scope as they don't include support for continious input. It can be found as the ```OnboardModuleState.java``` file

<div id="swerve"></div>

# Swerve Subsystem

Once the swerve module subsystem is created, the next part is creating the swerve subsystem. The swerve subsystem consists of a gyroscope, an array of swerve modules, and a swerve odometry object.

The first step is to declare everything for the swerve subsystem. This includes
* Swerve Modules: An array of swerve modules
* Gyro: The gyroscope used on the robot, reads the robots orientation
* Gyro Offset: The offset of the gyroscope to zero it.
* Swerve Odometry: The odometry of the robot. This tracks the wheel positions and angles to know the pose of the robot

Next is to initialize everything in the Swerve contructor. Initialize the gyro first as that is used in other object's initializations. After you initialize the gyro, it needs to be zeroed. 

```
public void zeroGyro(){
    gyroOffset = gyro.getYaw().getValueAsDouble();
}

public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(getYaw());
}

public double getYaw(){
    return (DriveConstants.GyroInvert)
      ? 180 - (gyro.getYaw().getValueAsDouble() - gyroOffset)
      : gyro.getYaw().getValueAsDouble() - gyroOffset;
}
```

When zeroing the gyro, the gyro offset is set to the current gyro position so when the method getYaw() is called, it returns 0 as the current heading.

```
swerveModules = 
    new SwerveModule[] {
      new SwerveModule(
          0,
          ModFL.name,
          ModFL.driveMotorID,
          ModFL.angleMotorID,
          ModFL.encoderID,
          ModFL.angleOffset),
      new SwerveModule(
          1,
          ModFR.name,
          ModFR.driveMotorID,
          ModFR.angleMotorID,
          ModFR.encoderID,
          ModFR.angleOffset),
      new SwerveModule(
          2,
          ModBL.name,
          ModBL.driveMotorID,
          ModBL.angleMotorID,
          ModBL.encoderID,
          ModBL.angleOffset),
      new SwerveModule(
          3,
          ModBR.name,
          ModBR.driveMotorID,
          ModBR.angleMotorID,
          ModBR.encoderID,
          ModBR.angleOffset)
      };
```

The next step is initializing all the swerve modules. This is an array of 4 swerve modules starting from Front Left, Front Right, Back Left, then Back Right. 

Each module is initialized using the different information defined in the constants file. Along with that, each module is given a module number (corresponding to its place in the array) to easily be able to call the module from the array.

After initializing all the modules, the angles will need reset so that the integrated angle encoder of each module is the same as the angle encoders. There is one bug that is known when resetting the modules that causes motors to invert. A solution to this is to add a delay before resetting the modules to ensure the bug does not happen.

```
swerveOdometry = 
      new SwerveDriveOdometry(
        DriveConstants.swerveKinematics, 
        getHeading(), 
        getModulePositions(),
        new Pose2d(0.0, 0.0, new Rotation2d(0.0, 0.0)));
```

The final part of the constructor is initializing the swerve odometry. The new pose created will be the initial robot pose. If the initial pose needs changed, here would be the place to change that. 

This could be used for autonomous to set the robot to a known pose if the robot is running an autonomous command from that pose

```
public void periodic() {
    swerveOdometry.update(getHeading(), getModulePositions());

    for (SwerveModule mod : swerveModules){
      SmartDashboard.putNumber(mod.name + " Encoder", mod.getCANcoder().getDegrees());
      SmartDashboard.putNumber(mod.name + " Integrated", mod.getPosition().angle.getDegrees());
    }

    SmartDashboard.putString("Robot Pose", getOdomPose().toString());
}
```

The next step is adding to the periodic method. This method runs once per scheduler (about every 20 ms). It is used to update values and display then on smart dashboard. The method updates the positions read by the swerveOdometry object, and displays the angles for the integrated and angle encoders of each module. The method also displays the current calculated robot pose from swerve odometry.

```
public void drive(
      Translation2d translation,
      double rotation,
      boolean fieldRelative,
      boolean isOpenLoop,
      boolean isLocked){
}
```

One of the most important methods is the drive method. This method takes different parameters and based off of those parameters, the swerve drive subsystem calculates a desired swerve module state and drives each swerve module to their desired states which in turn drives the robot. 

The first part is the parameters. The drive method takes a Translation2d which is an x and y vector on where the robot wants to go. The second parameter is the rotation or how much the robot should rotate. The field relative boolean controls whether the robot drives relative to the field where a direction on the field is forward or non field relative where forward for the robot is the front of the robot. The next parameter controls whether the drive for the swerve modules is open loop or not. Finally the isLocked boolean controls whether the swerve drive should lock in place or keep driving. 

```
    if (isLocked){
      // Sets the modules to drive inwards towards the center of the robot to lock the robot in place.
      final SwerveModuleState[] swerveModuleStates =
          new SwerveModuleState[] {
            new SwerveModuleState(0.2, Rotation2d.fromDegrees(225)),
            new SwerveModuleState(0.2, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0.2, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0.2, Rotation2d.fromDegrees(45))
          };

      for (SwerveModule mod : swerveModules){
        mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
      }
    } 
```

If the isLocked boolean is true, the robot should cross its wheels and stay stopped. This method creates a new SwerveModuleState each with a speed and rotation for each swerve module. With the rotation, each wheel points inward and drives inward which keeps the robot stopped and in place. The little bit of active drive to drive the robot inward keeps other robots from being able to push the robot around when stopped.

After each SwerveModuleState is created, a for loop is used to sent each state to its corresponding module where each module is set to the desired state.

```
    else {
      final ChassisSpeeds chassisSpeeds;

      
      if (fieldRelative){
        chassisSpeeds = 
          ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(), translation.getY(), rotation, getHeading());
      } else {
        chassisSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
      }

      final SwerveModuleState[] swerveModuleStates = 
        DriveConstants.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MaxSpeed);

      for (SwerveModule mod : swerveModules){
        mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
      }
    }
```

The second part of the drive method is if the robot is not locked. When the robot is not locked, the translation and rotation parameters will need calculated into SwerveModuleStates for each module. 

The first part in calculating the SwerveModuleStates is to initialize the centerOfRotation of the robot and the chassisSpeeds. The chassisSpeeds represents the robot's velocity

if the robot is field relative, then the chassis speeds will be calculated using the gyroscope to make the speeds field relative, without a gyro, the speeds can only be calculated with the robot being non field relative. 

After calculating the chassisSpeeds, using the  swerveKinematics defined in Constants, the chassisSpeeds can be converted to SwerveModuleStates.

The SwerveModuleStates are then desatureted. If one or more SwerveModuleStates have a speed higher than the max speed, this method readjusts the speeds so that the highest speed is set to the max speed. It readjust all the other speeds so that the states keep the same ratio of speeds when compared to each other.

Finally, after the SwerveModuleStates are created, the states are sent to the modules using a for-loop.

<div id="command"></div>

# Swerve Drive Command

After the swerve subsystem is created, there needs to be a command to take the joystick inputs and send them to the drive method of the swerve subsystem. 

The first step is creating a command. This can be called the SwerveDrive command

The command will take a few parameters to be created. It will need:
* Swerve: The swerve subsystem object
* Translation Supplier: The x axis joystick input
* Strafe Supplier: The y axis joystick input
* Rotation Supplier: The rotation joystick input
* Field Orientated: The boolean telling whether the robot should drive field orientated
* Is Locked: The boolean telling whether the robot should lock its wheels or not

For this command, the parameters will need declared, along with 2 two SlewRateLimiters for the x axis and y axis. 

In the contructor, everything is initialized and the SlewRateLimiters are initilized using the constands defined in Operator Constants.

In the contructor, after initilizing the swerve subsystem object, the addRequirements method needs to be called. This method makes it so when the SwerveDrive command is called, any other command using the swerve subsystem will stop and SwerveDrive command will use the swerve subsystem. 

Within the execute method of the command, which is the method that will be ran once per schduler of the command (about every 20ms), there are about 4 key components that make this command work. 

```
double xAxis = 
      MathUtil.applyDeadband(m_translationSup.getAsDouble(), OperatorConstants.stickDeadband);
double yAxis = 
      MathUtil.applyDeadband(m_strafeSup.getAsDouble(), OperatorConstants.stickDeadband);
double rAxis = 
      MathUtil.applyDeadband(m_rotationSup.getAsDouble(), OperatorConstants.stickDeadband);
```

The first step is to apply the joystick deadband to the joystick inputs. This creates a percentage of the joystick's input that returns zero or nothing. Generally the deadband value is kept between %10-%15 which means for the first %10-%15 of the joystick input, the output will be zero or nothing. This is done so that the controls are not so touchy that the robot is hard to control or stop. 

```
double xAxisSquared = xAxis * xAxis * Math.signum(xAxis);
double yAxisSquared = yAxis * yAxis * Math.signum(yAxis);
```

The next step is optional, but does help with controlling the robot and its acceleration. By squaring the inputs, the output will get exponentially larger as the input. This allows for finer control with small movements, but greater acceleration with larger inputs. If the input is not squared, the output will be linear to the inputs and there will be less control on the larger and smaller ends of the input. 

```
double xAxisFiltered = m_xAxisLimiter.calculat(xAxisSquared);
double yAxisFiltered = m_yAxisLimiter.calculat(yAxisSquared);
```

The SlewRateLimiters use the value their initlized with to calculate an output that is limited to change by no more than that value. This creates a max acceleration for the robot or how fast you want the speed of the robot to change. A small rate limit makes it hard for the robot to stop, but too large of a rate limit makes it hard to have small control and fine movements. It is a balance between these values that the rate limit should be. It will take testing and driving, and its up to the driver how fast they want their robot to accelerate or what is safe for the field/testing field.

```
swerve.drive(
      new Translation2d(-xAxisFiltered, -yAxisFiltered).times(DriveConstants.MaxSpeed),
      -rAxis * DriveConstants.MaxAngularSpeed,
      m_fieldOrientated.getAsBoolean(),
      true,
      m_isLocked.getAsBoolean());
```

Finally, everything is sent into the swerve subsystem's drive method which creates and sends all the SwerveModuleStates to each individual wheel which moves the robot. 

Depending on the direction of the joystick, the inputs will need to have a negative sign. This could be one problem when first testing. If the robot does not turn the correct direction, or more opposite of where it is supposed to, the joystick inputs may need inverted.

A new translation is created with the x and y values and it is scaled by the max speed. The rotation value is scaled with the angular speed. Then the field orientated and is Locked booleans get suppield and the method is called. 

For teleoperated, the isOpenLoop can be left as true. Generally, the robot is only closed loop in some autonomous, but not always.

<div id="init"></div>

# Adding and Initializing Swerve Drive

The final step to having swerve drive running is creating and calling everything when the robot is initilized. The RobotContainer class is used to contruct all the robot's neccessary subsystems commands, buttons, and joysticks

```
private final XboxController driver = new XboxController(OperatorConstants.driverPort);

private final int translationAxis = XboxController.Axis.kLeftY.value;
private final int strafeAxis = XboxController.Axis.kLeftX.value;
private final int rotationAxis = XboxController.Axis.kRightX.value;
```

The first step if not done already is to initialize the driver controller and the axes for the translation, strade, and rotational values of the joystick. 

```
private final JoystickButton zeroGyro = 
    new JoystickButton(driver, XboxController.Button.kX.value);
private final JoystickButton isLocked = 
    new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
private final JoystickButton invertGyro = 
    new JoystickButton(driver, XboxController.Button.kY.value);
```

The next step is to initialize buttons used with the swerve drive. The zeroGyro button and isLocked button are neccessary, but the invertGyro button is optional. The first part is defining which controller the button belongs to while the second part defines what button on the controller will be used.

```
private final Swerve swerve = new Swerve();
```

In order to use the swerve subsystem. It will need initlialzed in Robot Container class. 

Within the constructor of the Robot Container class, the default commands, and button/command bindings can be declared.

```
swerve.setDefaultCommand(
      new SwerveDrive(
        swerve, 
        () -> driver.getRawAxis(translationAxis), 
        () -> driver.getRawAxis(strafeAxis), 
        () -> driver.getRawAxis(rotationAxis), 
        () -> true, 
        () -> isLocked.getAsBoolean()));

configureBindings();
```

Each subsystem can have its own default command. The easiest way to run the Swerve Drive command is to set it as the default command for the swerve subsystem. This is where all the values from the driver controller's axis can be gotten. Almost all robots drive field orientated for competition as it is very hard to drive non-field orientated with a swerve robot. 

After setting the default command, the configureBindings() method is called which sets all the commands for the buttons initialized above

```
zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));

invertGyro.onTrue(new InstantCommand(() -> swerve.invertGyro()));
```

Within the configureBindings method, new commands are created to be attached to the buttons. Buttons have a couple of different triggers that can call the comands. There is onTrue and onFalse that run the command when the button changes from false to true and vice versa. The other set is toggleOnTrue and toggleOnFalse which toggles the command on or off depending on the value. 

In this case, because the methods only need to be called once, an InstantCommand is created as it only runs once. 

After that, everything is done for creating a swerve drive robot. Check back through the code to confirm every method is created and all the constants are defined. Ensure all the IDs are set. 

Once everything is done, the swerve drive should work when connecting to the driver station. 

Make sure to find and set the encoder offsets for each module. This can be done by turning on the robot and pulling up the driver station so that the encoder values can be read. Turn all the wheels of the robot so that they face the front or foward of the robot with the bevel gear or the gear on the wheel facing the right. Use a straight edge to alight the sides of the wheels to each other so that each side of the robot is straight and the front and back wheels are aligned with each other. Once all the wheels are aligned, read the encoder values for the module, and those values are the new encoder offsets.

<div id="problem"></div>

# Common Problems with Swerve

Swerve is complex and there are a lot of problems that could happen. Here are some common problems and ideas or solutions for those problems

## Field orientated is not working

Field orientated can be checked by driving the robot forward, then turing it and driving it forward again, if the robot is field orientated, then it will drive in the same direction both times. If the robot is not driving in field orientated then that means there is no value being read from the gyroscope. Ensure the gyro is called and created correctly and that the value of the gyro is outputting and getting updated when rotating the robot. 

Another issue is not having the correct gyro id. The id of a gyro should be 0, however it can be set manualy using the gyro's brand's config program or tool/hardware client.

Make sure the gyro's signal wires are correctly plugged in and connected. Ensure the power wires are connected and the gyro is recieving power. 

## One or multiple swerve modules are not driving/facing the correct direction

If one or more swerve modules are not facing the correct direction, or they drive the wrong way, then the most likely issue is the encoder offsets for the swerve modules are not correct or are being converted incorrectly. 

To fix this, the first step is to reset the swerve modules. 

If that does not work, then there is a problem of converting the module's offset in constants, to applying the constant when resetting to absolute. Check to make sure the unit conversions are correct. The offfset should be printed and measured in degrees where it is converted to a rotation2d. Then that rotation 2d is used for resetting the integrated encoder to the correct value by getting degrees from the rotation2d with the method getDegrees(). 

If this does not work. Make sure each encoder has the correct ID and that they are matched to their correct module. Each encoder should have a hardware client that can be used to configure the IDs if their incorrect or need changed. 

## The swerve modules constantly spin and never stop

If the swerve modules constantly spin or drive and never stop, this means that the setSpeed() and setAngle() methods of the swerve module class cannot reach their endpoint. 

The first step is to make sure the output is going in the intended direction. If the output of the module is going in the opposite direction of the intended output, then the wheel will spin indefinetly. 

If this doesn't work, check the PID controllers of the swerve module subsystem and ensure that they function correctly. 

## How do I set the PID constants

A PID controller is used to control and set the speed and angle for each swerve module. The three letters stand for proportional, integral, and derivitive. The P or proportional is what we'll look at. The P means that it is a proportional constant to the output. If the output needs to turn 180 degrees and the P is 1, then the output will be 180 which is not good as motors only take a input form 0-1. To change this, the P can be lowered. 

The P constant is a great way to proportionally add or take away power or output from the motor. If the modules are not rotating enough and can reach their end target, then the P constant will need raised to a larger number. If the modules are alternating back and forth and consistently overshooting their intended angle, then the P constant is too high and will need lowered. 

<div id="words"></div>

# Key Words and Names

Here are some key words, names, or ideas that are commonly used for swerve drive

* Declaring: Creating an object or variable. Ex: int number;

* Initializing: Giving an object or variable a value for the first time. Ex: number = 3;

* Jittering: The process where the turning of the swerve module moves back and forth quickly to move very miniscule amounts and creates alot of sound. Tuning PID coeffiencts can help reduce the amount of jittering

* Deadband: The percentage of the joystick's input that is set to nothing or zero

* Track Width and Wheel Base: The x and y distance between the center of the wheels for the swerve chassis. These are the same for a square chassis

* SwerveDriveKinematics: A helper class that is initialized with the location of each swerve module in relation to the center of the robot, so this helper class can help convert a robot chassis's speed to the state of each Swerve Module

* Translation: A translation or vector from a point. A magnitude of direction and the angle of that direction. 

* Encoder: A device that is seperate or attached/internal to a motor that measures the rotations of a motor or axle

* Motor: A device that when supplied with a signal from a motor controller, and supplied with power, will rotate at a set percentage or rpm in a counter clock wise or clock wise direction

* Motor Controller: A device that controls a motor with a signal. This device helps regulate the power of a motor to spin only when directed. Motors are never initialized in code, but motor controllers are.

* Odometry: A process or object that tracks wheel positions and rotations, and knowing the position of each wheel, the odometry calculates the pose of the robot or how far it as traveled in a direction/the location of the robot

* Gyroscope: A device that is used to measure the robots orientation in terms of three angles: yaw, pitch, and roll. If a robot is flat on the floor and facing forward, the yaw is the left right movement or the angle the robot is facing. The pitch is the up and down movement of the front or back of the robot. The roll is the up and down movement of the left or right side of the robot. 