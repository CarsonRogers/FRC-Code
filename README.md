# FRC Code Documentation

Here are some resources to help with programming a robot. These resources include swerve drive, photon vision, intakes, arms, commands, autonomous, and path planner

<details>
    <summary>Content</summary>
    <ol>
        <li><a href="#swerve">Swerve Drive</a></li>
        <li><a href="#intake">Intakes, Shooters, and Feeders</a></li>
        <li><a href="#arm">Arms and Rotation</a></li>
        <li><a href="#commands">Creating Commands</a></li>
        <li><a href="#auton">Setting Up Autonomous</a></li>
        <li><a href="#photon">Photon Vision</a></li>
        <li><a href="#planner">Path Planner</a></li>
    </ol>
</details>

<div id="swerve"></div>

# Swerve Drive

Swerve drive is a complex drive system where there are usually 4 wheels and each wheel is controlled individually by 2 motors where one controls the speed, and the other controls the rotation or heading of the wheel. 

Swerve drive is a very common drive base that is fast and agile, and seen a lot with high end competitive FRC bots. 

To learn more and see an example of swerve drive, click [here](SwerveCodeExample/ReadME.md).

<div id="intake"></div>

# Intakes, Shooters, and Feeders

Most intakes, shooters, and feeders follow the same system of motors that are set to either a certian percentage of speed, or controlled by PID controllers. This example uses percentage controlled speed. 

Many intakes and feeders use 1-2 motors to control the subsystem, along with an occasional sensor to detect game pieces. 

Shooters use generally use 2 or 4 motors to get enough speed to shoot the game pieces. The relative encoders of the shooter motors are used to get the RPM of the shooters.

To learn more and see an exmaple of an intake, shooter, feeder, and an intake/shooter, click [here](IntakeShooterFeederExample.ReadME.md).

<div id="arm"></div>

# Arms and Rotation

<div id="commands"></div>

# Creating Commands

<div id="auton"></div>

# Setting Up Autonomous

<div id="photon"></div>

# Photon Vision

Photon Vision is a vision processing system that is used to identify objects or april tags on the field. The system uses one or more cameras and a coprocessor to filter the images and determine poses and angles for the robot to aim at.

This vision processing really excells with multiple cameras on a robot. In 2024, 3 cameras and a orange pi were used for vision processing.

To learn more and see an example of photon vision set up, click [here](PhotonVisionExample/ReadME.md). 

This example consists of two cameras and assumes there is a coprocessor like an orange or raspberry pi being used. It walks through the basics of filtering the camera inputs and estimating the robot's pose. 

This also assumes that the robot is running swerve drive. See above to learn swerve drive.

<div id="planner"></div>

# Path Planner

Path Planner is a tool to generate motion profiles for FRC robots. This can be used to help create different autonomous routes for the robot. 

Each motion profile can be generated from a series of points. The user can pick where the robot should move, and pathplanner will calculate the speed and rotation for the robot for each point. 

To learn more and see an example of Path Planner set up, click [here](PathPlannerExample/ReadME.md).

This also assumes that the robot is running swerve drive. See above to learn swerve drive. 

For more information on Path Planner, click [here](https://pathplanner.dev/home.html).