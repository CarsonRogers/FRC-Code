package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants.ModBL;
import frc.robot.Constants.ModuleConstants.ModBR;
import frc.robot.Constants.ModuleConstants.ModFL;
import frc.robot.Constants.ModuleConstants.ModFR;

public class Swerve extends SubsystemBase {
  private final SwerveModule[] swerveModules;
  private final Pigeon2 gyro;
  private final SwerveDriveOdometry swerveOdometry;

  /** The offset value for the gyro */
  private double gyroOffset;

  /**
   * Represents a swerve drive subsystem.
   * This class initializes a gyro and odometry
   * It provides methods to access the swerve drive system and its state
   */
  public Swerve() {
    gyro = new Pigeon2(DriveConstants.GyroID);
    gyro.getConfigurator().apply(new Pigeon2Configuration());
    zeroGyro();

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

    /**
     * By pausing for a second before setting the module offsets, 
     * a bug is avoided with inverting motors
     * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
     */
    Timer.delay(1.0);
    resetModulesToAbsolute();

    /**
     * Creates a new instance of swerve drive odometry
     * This is responsible for handling and tracking the wheel positions
     * 
     * Initialization includes:
     *  - the swerveKinematics object which is the kinematic property of the swerve drive
     *  - The method getHeading() which returns the current orientation of the robot
     *  - getModulePositions() which returns an array of all the module positions
     *  - A new Pose2d which is the initial robot's pose
     */
    swerveOdometry = 
      new SwerveDriveOdometry(
        DriveConstants.swerveKinematics, 
        getHeading(), 
        getModulePositions(),
        new Pose2d(0.0, 0.0, new Rotation2d(0.0, 0.0)));


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
  }

  /** 
   * This method is called periodically (once per scheduler run) to update the state of the robot
   * The swerve odometry is updated
   * Any relevant data is displayed using SmartDashboard
   */
  @Override
  public void periodic() {
    swerveOdometry.update(getHeading(), getModulePositions());

    for (SwerveModule mod : swerveModules){
      SmartDashboard.putNumber(mod.name + " Encoder", mod.getCANcoder().getDegrees());
      SmartDashboard.putNumber(mod.name + " Integrated", mod.getPosition().angle.getDegrees());
    }

    SmartDashboard.putString("Robot Pose", getOdomPose().toString());
  }

  /**
   * Drives the swerve drive system based on a given translation and rotation values
   * 
   * @param translation    The translation of the robot in x and y cordinates
   * @param rotation       The rotation of the robot
   * @param fieldRelative  True if the translation and rotation are field relative, otherwise false
   * @param isOpenLoop     True if the drive should be performed in open loop mode, false for closed
   * @param isLocked       True if the swerve drive system is locked in place, false otherwise
   */
  public void drive(
      Translation2d translation,
      double rotation,
      boolean fieldRelative,
      boolean isOpenLoop,
      boolean isLocked){
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
    } else {
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
  }

  /**
   * @return The heading of the robot as a Rotation2d object
   */
  public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(getYaw());
  }

  /**
   * @return The yaw angle of the robot in degrees
   */
  public double getYaw(){
    return (DriveConstants.GyroInvert)
      ? 180 - (gyro.getYaw().getValueAsDouble() - gyroOffset)
      : gyro.getYaw().getValueAsDouble() - gyroOffset;
  }

  /**
   * @return The pitch angle of the robot in degrees
   */
  public double getPitch(){
    return gyro.getPitch().getValueAsDouble();
  }

  /**
   * @return The roll angle of the robot in degrees
   */
  public double getRoll(){
    return gyro.getRoll().getValueAsDouble();
  }

  /**
   * @return the current pose of the swerve subsystem
   */
  public Pose2d getOdomPose(){
    return swerveOdometry.getPoseMeters();
  }

  /**
   * @return An array of the positions of all swerve modules
   */
  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : swerveModules){
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  /**
   * Retrieves the current state of all swerve modules
   * 
   * @return an array of SwerveModuleState objects representing the state of each swerve module
   */
  public SwerveModuleState[] getModuleStates(){
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : swerveModules){
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  /** Resets the gyro offset to the current yaw value */
  public void zeroGyro(){
    gyroOffset = gyro.getYaw().getValueAsDouble();
  }

  /** Inverts the gyro by adding or subtracting 180 to the gyro offset */
  public void invertGyro(){
    if (gyroOffset > 180){
      gyroOffset -= 180;
    } else gyroOffset += 180;
  }

  /** Resets all swerve modules to their absolute position */
  public void resetModulesToAbsolute(){
    for (SwerveModule mod : swerveModules){
      mod.resetToAbsolute();
    }
  }

  public void resetPose(Pose2d pose){
    swerveOdometry.resetPosition(getHeading(), getModulePositions(), pose);
  }
}