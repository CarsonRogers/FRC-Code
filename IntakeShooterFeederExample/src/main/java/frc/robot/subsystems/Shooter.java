// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  private CANSparkMax rightShooterMotor;
  private CANSparkMax leftShooterMotor;

  private CANSparkMax auxRightShooterMotor;
  private CANSparkMax auxLeftShooterMotor;

  private RelativeEncoder shooterEncoder;
  private RelativeEncoder auxShooterEncoder;

  /**
   * Creates a new Shooter subsystem with four motors. Two primary motors and two auxiliary motors
   * In this example, the two sets of motors run on the same axle, so only one encoder from each pair of 
   * motors is needed. An encoder can be created for each motor and then the values can be averaged out if wanted
   */
  public Shooter() {
    rightShooterMotor = new CANSparkMax(ShooterConstants.rightShooterID, MotorType.kBrushless);
    leftShooterMotor = new CANSparkMax(ShooterConstants.leftShooterID, MotorType.kBrushless);
    configShooterMotors();

    auxRightShooterMotor = new CANSparkMax(ShooterConstants.auxRightShooterID, MotorType.kBrushless);
    auxLeftShooterMotor = new CANSparkMax(ShooterConstants.auxLeftShooterID, MotorType.kBrushless);
    configAuxShooterMotors();

    shooterEncoder = rightShooterMotor.getEncoder();
    auxShooterEncoder = auxRightShooterMotor.getEncoder();
  }

  /**
   * Sets the shooting speed from constants to the shooting motors
   * The left motor of each motor pair is set to follow the right motor so
   * only the right motor speed needs set
   */
  public void shoot(){
    rightShooterMotor.set(ShooterConstants.shootSpeed);
    auxRightShooterMotor.set(ShooterConstants.auxShootSpeed);
  }

  /**
   * Steps each motor and sets their output to 0
   */
  public void stop(){
    rightShooterMotor.set(0);
    leftShooterMotor.set(0);
    auxRightShooterMotor.set(0);
    auxLeftShooterMotor.set(0);
  }

  /**
   * Returns the RPM of the main shooter motors from the right side's encoder
   * @return the RPM of the main shooter motor pair
   */
  public double getRPM(){
    return shooterEncoder.getVelocity();
  }

  /**
   * Returns the RPM of the aux shooter motors from the right side's encoder
   * @return the RPM of the aux shooter motor pair
   */
  public double getAuxRPM(){
    return auxShooterEncoder.getVelocity();
  }

  /**
   * Configures the right and left shooter motors by resetting them and 
   * setting all necessary settings declared in Constants
   */
  private void configShooterMotors(){
    rightShooterMotor.restoreFactoryDefaults();
    leftShooterMotor.restoreFactoryDefaults();

    rightShooterMotor.enableVoltageCompensation(Constants.voltageComp);
    leftShooterMotor.enableVoltageCompensation(Constants.voltageComp);

    rightShooterMotor.setInverted(ShooterConstants.invertRightShooter);
    leftShooterMotor.setInverted(ShooterConstants.invertLeftShooter);

    rightShooterMotor.setIdleMode(ShooterConstants.rightShooterIdleMode);
    leftShooterMotor.setIdleMode(ShooterConstants.leftShooterIdleMode);

    leftShooterMotor.follow(rightShooterMotor);
  }

  /**
   * Configures the aux right and aux left shooter motors by resetting them and 
   * setting all necessary settings declared in Constants
   */
  private void configAuxShooterMotors(){
    auxRightShooterMotor.restoreFactoryDefaults();
    auxLeftShooterMotor.restoreFactoryDefaults();

    auxRightShooterMotor.enableVoltageCompensation(Constants.voltageComp);
    auxLeftShooterMotor.enableVoltageCompensation(Constants.voltageComp);

    auxRightShooterMotor.setInverted(ShooterConstants.invertAuxRightShooter);
    auxLeftShooterMotor.setInverted(ShooterConstants.invertAuxLeftShooter);

    auxRightShooterMotor.setIdleMode(ShooterConstants.auxRightShooterIdleMode);
    auxLeftShooterMotor.setIdleMode(ShooterConstants.auxLeftShooterIdleMode);

    auxLeftShooterMotor.follow(auxRightShooterMotor);
  }
}
