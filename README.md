# FRC Code Documentation

Here are some resources to help with programming a robot. These resources include swerve drive, photon vision, intakes, arms, commands, autonomous, and path planner

<details>
    <summary>Content</summary>
    <ol>
        <li><a href="#swerve">Swerve Drive</a></li>
        <li><a href="#intake">Intakes and Shooters</a></li>
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

To learn more and see an example of swerve drive, click [here](SwerveCodeExample/ReadME.md)

<div id="intake"></div>

# Intakes and Shooters

<div id="arm"></div>

# Arms and Rotation

<div id="commands"></div>

# Creating Commands

<div id="auton"></div>

# Setting Up Autonomous

<div id="photon"></div>

# Photon Vision

Photon Vision is a vision processing system that is used to identify objects or april tags on the field. The system uses one or more cameras and a coprocessor to filter the images and determine poses and angles for the robot to aim at

This vision processing really excells with multiple cameras on a robot. In 2024, 3 cameras and a orange pi were used for vision processing

To learn more and see an example of photon vision set up, click [here](PhotonVisionExample/ReadME.md). 

This example consists of two cameras and assumes there is a coprocessor like an orange or raspberry pi being used. It walks through the basics of filtering the camera inputs and estimating the robot's pose. 

This also assumes that the robot is running swerve drive. See above to learn swerve drive.

<div id="planner"></div>

# Path Planner