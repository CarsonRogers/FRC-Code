// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private CANSparkMax intakeMotor;

  /**
   * Creates a new basic Intake subsystem
   * This Intake consists of one intake motor controlling the intake of a game piece
   */
  public Intake() {
    intakeMotor = new CANSparkMax(IntakeConstants.intakeID, MotorType.kBrushless);
    configIntakeMotor();
  }

  /**
   * An intake method that when called, calls the other intake method with the defined
   * intake speed declared in constants. This method is basically acts as a 
   * default for the other intake method. If no speed is declared in the other intake method, 
   * then this method will run with the intake speed in constants
   */
  public void intake(){
    intake(IntakeConstants.intakeSpeed);
  }

  /**
   * Sets the percentage output of the intake motor
   * @param speed the percentage output of the intake motor from 0 to 1
   */
  public void intake(double speed){
    intakeMotor.set(speed);
  }

  /**
   * Stops the intake motor from moving.
   */
  public void stop(){
    intakeMotor.set(0);
  }

  /**
   * Configures the intake motor by restoring to 
   * defaults then changing the necessary settings defined in Constants
   */
  private void configIntakeMotor(){
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(IntakeConstants.invertIntake);
    intakeMotor.setIdleMode(IntakeConstants.intakeIdleMode);
  }
}
