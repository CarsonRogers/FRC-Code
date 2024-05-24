// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeShooter extends SubsystemBase {
  /** Creates a new IntakeShooter. */

  


  /**
   * Creates a new IntakeShooter class that combines the functions of an intake and a shooter
   * This was used for our 2024 robot Jellybean where game pieces where picked up 
   * and shot from the same mechanism. That robot used CANSparkFlexes which are code wise almost exactly the same
   * compared to CANSparkMaxes. This example uses CANSparkMaxes
   */
  public IntakeShooter() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
