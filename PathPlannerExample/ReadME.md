# Creating Swerve Drive
## A swerve drive subsystem with code from FRC 461 and FRC 3175
<details>
    <summary>Content</summary>
    <ol>
        <li><a href="#what">What is Path Planner</a></li>
        <li><a href="#start">Getting Started</a></li>
        <li><a href="#swerve">Setting Up Swerve Drive</a></li>
        <li><a href="#command">Creating Named Commands</a></li>
        <li><a href="#path">Generating Paths and Autos</a></li>
        <li><a href="#robot">Configuring Robot Container</a></li>
    </ol>
</details>

<div id="what"></div>

# What is Path Planner

Path Planner is a tool created for FRC robots that generates motion profiles based off of selected points

Motion profiles can be created into paths in which paths can be linked together along with different named commands to create an autonomous for a robot to run. 

Path Planner has automatic file saving where any changes to files inside the Path Planner program will automatically update in the folder were their stored. ;

<div id="start"></div>

# Getting Started

The Getting Started page for Path Planner can be found on its website [here](https://pathplanner.dev/gui-getting-started.html).

The basics of Path Planner include:
* Installing Path Planner from the Microsoft Store, Mac Store, or the manual install
* Opening a Robot Project and setting up the file path/robot settings
* Creating Paths and Autos

Once Path Planner is installed, the next step is to open the program and select the code folder containing the robot code that you want to use Path Planner in. 

Once that is selected, the settings tab can be navigated to by using the side menu. Make sure the robot's width, length, and speed contraints match the contraints set in Constants. For testing path planner, it could be useful to lower the speed to ensure the robot works at lower speeds before moving to speed up the auto. This ensures the envoirment is safe and there is more reaction time to stop the robot if undesired movements take place. 

The field image can also be set in the settings menu. This image is not always 100% accurate, however it works relatively well. 

The navigation grid can be used to block out certian areas to avoid field obstacles. 

<div id="swerve"></div>

# Setting Up Swerve Drive

```
    /**
     * Configures the Path Planner's AutoBuilder for holonomic type drive trains (includes swerve)
     * This is responsible for controlling the drive subsystem when a path is ran
     * 
     * Initialization includes:
     *  - the supplier function to get the robot's pose, in this case the pose is from odometry, but it can be from vision
     *  - the consumer function to reset the robot's pose
     *  - the suppier function to get thr robot relative ChassisSpeeds
     *  - the consumer function tp set the robot relative ChassisSpeeds
     *  - the config for the Holonomic Path Follower
     *  - the boolean supplier to determine whether the path should be flipped or not. All paths are made on the blue side
     * if the path needs ran on the red side, then it will need flipped
     *  - the swerve drive subsystem
     */
    AutoBuilder.configureHolonomic(
      this::getOdomPose, 
      this::resetPose, 
      () -> DriveConstants.swerveKinematics.toChassisSpeeds(getModuleStates()), 
      speeds -> {
        // Method that will drive the robot given robot relative ChassisSpeeds
        SwerveModuleState[] swerveModuleStates = 
          DriveConstants.swerveKinematics.toSwerveModuleStates(speeds);
      }, 
      DriveConstants.SwerveHolonomicPathFollowerConfig, 
      () -> {
        //Boolean supplier that controls when the path should be mirrored for the red alliance
        //This will flip the path being followed to the red side of the field.
        //The Origin will remain on the blue side
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()){
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      }, 
      this);
```

Only one addition is needed for swerve drive to work with path planner. In the constructor for the swerve drive subsystem. the configureHolonomic method needs called. Within this method are a few key components

The first parameter is the method to get the robot's pose. This can be gotten from the odometry or if the robot is using vision, it can be from the pose estimator. However, it is sometimes better to have autonomous on pure odometry as vision may cause errors with poses and pathing if not everythin is completly accurate.

The second parameter resets the robot's pose to the inputted pose. 

The third parameter supplies the swerve module states given that the robot is field relative

The fourth parameter takes the given swerve module states and drives the modules 

```
public static final HolonomicPathFollowerConfig SwerveHolonomicPathFollowerConfig = 
            new HolonomicPathFollowerConfig(
                new PIDConstants(
                    AutoConstants.AutoDriveD, AutoConstants.AutoDriveI, AutoConstants.AutoDriveD),
                new PIDConstants(
                    AutoConstants.AutoTurnP, AutoConstants.AutoTurnI, AutoConstants.AutoTurnD),
                AutoConstants.MaxSpeed,
                CenterToWheel,
                new ReplanningConfig(
                    true, true)
            );
```

The fifth parameter is the configuration for the holonomic drive. This can be either created in the method, or in constants. This example has the config in constants. The config consist of setting the max speed, turning and drive PIDs, and getting the center to the wheel

The sixth parameter determines whether the path needs flipped. Because all paths are created on the blue alliance side, if the alliance is red, then the path needs flipped.

The seventh and last parameter is the actual swerve drive subsystem.

<div id="command"></div>

# Creating Named Commands

Named commands are used to call different robot commands from the code to Path Planner. 

```
private void registerAutonCommands(){

    /* A generic command to showcase how named commands are registers. They require a name and a command. 
     * This can be used for intake and rotation commands to fully control your robot from path planner. 
     * The names will not be visible within path planner when creating named commands and they must be 
     * exactly the same when inputted in Path Planner. 
     */
    NamedCommands.registerCommand(
      "Reset Gyro", new InstantCommand(() -> swerve.zeroGyro()));
}
```

To keep everything organized, all the named commands are registered in a new method. This method is called within the RobotContainer Contructor and called before the auto chooser.

The basic idea of named commands is the name and the command. The name is the name used in Path Planner to call the commands to be used within different autos. The command part is the actual command that will be called when the command is called in Path Planner

<div id="path"></div>

# Generating Paths and Autos

Within Path Planner, different paths and autos can be created. Paths are motion profiles generated from a list of points and rotations, while autos are created from many paths and commands linked together. 

The main menu of Path Planner is seperated into two parts. Paths and Autos. The path section on the left side is where different paths can be created. It is important to create folders and name these paths so that when there are multiples of paths created, it is easy to find and track back specific paths later. 

When creating a new path, the first thing that will appear will be two points. A green point and a red point. These two points signifiy the start and end of the path. Each point also has a rotation that can be set. These points can be dragged to the desired spot to create a path. It is important to not create any harsh movements or actions. Path Planner has a hard time doing any tight rotations or complex movments, so the simplier the movements the better. 

Waypoints can be added between paths so that if one path is edited, another point on another path will be moved or editid to the same position. This is useful for linking the start and ends of different paths together. 

Different paths and named commands can be added together to create autos. 

The first step is to set the starting position. Using a preset starting pose, the robot will move to the starting pose of the first path, then continue from there. The easiest way to get the robot to do the path where ever it is is to set the preset pose to where ever the start of the first path is. That way, the robot will just start at the first path. 

All the paths and commands are ran in a sequential command group. Hitting the plus button is where you can select what to add to the auto. These include:
* Follow Path: A created path for the robot to follow
* Named Command: The name of the named command as initialized in robot code. This is how to run robot functions besides moving, like intakes and shooters
* Wait Command: A time in seconds for the robot to wait before continuing on
* Sequential Command Group: A group of commands and paths that run in a sequential order. This means that the first command or path will run completly before starting the next then the next and so on until every command or path has been ran in order. The Sequential Command Group ends when the last command has finished
* Parallel Command Group: A group of commands and paths that will run at the same time. For example, an intake command and a movement path can move at the same time. The Parrallel Command Group ends when every command has finished. 
* Parallel Deadline Group: A Parallel Command Group that will end when the deadline command finishes. The deadline command is the first command in the group. 
* Parallel Race Group: A Parallel Command Group where whenever the first command ends and finishes is when all the commands end and finish.

These different parts are the tools that make up each auto. For example, for a robot to shoot, then move and intake at the same time. The named shoot command would be called, followed by a parallel command or parallel race group with the movement path and the intake command inside. If the race group is used, then the robot will finish moving and continue, or will intake a note and continue. 

<div id="robot"></div>

# Configuring Robot Container

The final step for the robot to run all these newly created autos is to add the auto chooser to Robot Container

```
private final SendableChooser<Command> autoChooser;
```

The first step is to declare a sendable chooser with the command type called autoChooser. This can be done near where all the other subsystems and buttons are initialized

```
autoChooser = AutoBuilder.buildAutoChooser();
SmartDashboard.putData("Auto Chooser", autoChooser);
```

The next step is to initialize the autoChooser and display the object on SmartDashboard. This will allow the user to select the desired autonomous from the dashboard. 

```
public Command getAutonomousCommand() {
    return autoChooser.getSelected();
}
```

The final step is to return the selected command from the autoChooser.