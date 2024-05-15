// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.OnboardModuleState;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  public int moduleNumber;
  public String name;
  private Rotation2d angleOffset;
  private Rotation2d lastAngle;
  private SwerveModuleState desiredState = 
    new SwerveModuleState(0.0, new Rotation2d());

  private CANSparkMax angleMotor;
  private CANSparkMax driveMotor;
  private CANcoder angleEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private RelativeEncoder driveEncoder;

  private SparkPIDController driveController;
  private SparkPIDController angleController;

  private final SimpleMotorFeedforward drivFeedForward = 
    new SimpleMotorFeedforward(
      DriveConstants.DriveKS, DriveConstants.DriveKV, DriveConstants.DriveKA);

  public SwerveModule(
      int moduleNumber, 
      String name,
      int driveMotorID,
      int angleMotorID,
      int encoderID,
      Rotation2d angleOffset) {
    this.moduleNumber = moduleNumber;
    this.name = name;
    this.angleOffset = angleOffset;

    /* Angle Encoder Config */
    angleEncoder = new CANcoder(encoderID);
    configAngleEncoder();

    angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
    integratedAngleEncoder = angleMotor.getEncoder();
    angleController = angleMotor.getPIDController();
    configAngleMotor();

    driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getPIDController();
    configDriveMotor();

    lastAngle = getState().angle;
  }

  /**
   * Sets the desired state of the SwerveModule
   * @param desiredState The desired state of the Swerve Module
   * @param isOpenLoop A boolean indicating whether the control mode is open loop or not
   */
  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);
    this.desiredState = desiredState;
    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  /**
   * Sets the speed of the swerve module
   * @param desiredState The desired state of the swerve module
   * @param isOpenLoop A boolean indicating whether the control mode is open loop or not
   */
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

  /**
   * Sets the angle of the swerve module to the desired state. 
   * If the speed is less then 1% of the max speed, the module will not rotate
   * to prevent jittering
   * @param desiredState The desired state of the swerve module
   */
  private void setAngle(SwerveModuleState desiredState){
    Rotation2d angle = 
      (Math.abs(desiredState.speedMetersPerSecond) <= (DriveConstants.MaxSpeed * 0.01))
      ? lastAngle
      : desiredState
        .angle;

    angleController.setReference(angle.getDegrees(), ControlType.kPosition);
    lastAngle = angle;
  }  

  /**
   * @return The current state of the swerve module
   */
  public SwerveModuleState getState(){
    return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
  }

  /**
   * @return The desired state of the SwerveModule
   */
  public SwerveModuleState getDesiredState(){
    return desiredState;
  }

  /**
   * @return The position of the swerve module
   */
  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
  }

  /**
   * Returns the angle of the swerve module
   * @return the angle of the swerve module as a Rotation2d object
   */
  private Rotation2d getAngle(){
    return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
  }

  /**
   * Calculates the rotation of the angle encoder in radians
   * The default value of the returned position is in rotations
   * @return the rotation of the angle encoder as a Rotation2d object
   */
  public Rotation2d getCANcoder(){
    return Rotation2d.fromRadians(angleEncoder.getAbsolutePosition().getValue() * 2 * Math.PI);
  }

  /**
   * Resets the integrated angle encoder to the absolute postion.
   * The absolute position is calculated by subtracting the angle offset from the current position
   * The current position is read by the CANcoder
   */
  public void resetToAbsolute(){
    double absolutePosition = getCANcoder().getDegrees() - angleOffset.getDegrees();
    integratedAngleEncoder.setPosition(absolutePosition);
  }

  /**
   * Configures the angle encoder for the swerve module
   * The default range for a CANcoder is 0-360 degrees
   * Sets the sensor direction to counter clockwise positive
   */
  private void configAngleEncoder(){
    CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();
    swerveCANcoderConfig.MagnetSensor.SensorDirection = 
      SensorDirectionValue.CounterClockwise_Positive;

    angleEncoder.getConfigurator().apply(swerveCANcoderConfig);
  }

  /**
   * Configures the angle motor for the swerve module
   * This method sets various configurations for the angle motor.
   * At the end of this method, the integrated angle encoder is reset to match the CANcoder
   */
  private void configAngleMotor(){
    angleMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
    angleMotor.setSmartCurrentLimit(DriveConstants.AngleContinuousCurrentLimit);
    angleMotor.setInverted(DriveConstants.AngleInvert);
    angleMotor.setIdleMode(DriveConstants.AngleIdleMode);
    integratedAngleEncoder.setPositionConversionFactor(DriveConstants.AngleConversionFactor);
    integratedAngleEncoder.setVelocityConversionFactor(
      DriveConstants.AngleVelocityConversionFactor);
    angleController.setP(DriveConstants.TurnP);
    angleController.setI(DriveConstants.TurnI);
    angleController.setD(DriveConstants.TurnD);
    angleController.setFF(DriveConstants.TurnF);
    angleMotor.enableVoltageCompensation(Constants.voltageComp);
    resetToAbsolute();
  }

  /**
   * Configures the drive motor for the swerve module.
   * This method sets various configurations for the drive motor.
   * At the end of this method, the drive encoder is reset.
   */
  private void configDriveMotor(){
    driveMotor.restoreFactoryDefaults(); 
    CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
    driveMotor.setSmartCurrentLimit(DriveConstants.DriveContinousCurrentLimit);
    driveMotor.setIdleMode(DriveConstants.DriveIdleMode);
    driveEncoder.setVelocityConversionFactor(DriveConstants.DriveVelocityConversionFactor);
    driveEncoder.setPositionConversionFactor(DriveConstants.DriveConversionFactor);
    driveController.setP(DriveConstants.DriveP);
    driveController.setI(DriveConstants.DriveI);
    driveController.setD(DriveConstants.DriveD);
    driveController.setFF(DriveConstants.DriveF);
    driveMotor.enableVoltageCompensation(Constants.voltageComp);
    driveEncoder.setPosition(0.0);
  }
}
