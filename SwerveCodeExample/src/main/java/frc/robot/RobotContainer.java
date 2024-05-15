// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Swerve;

public class RobotContainer {

  /* Controllers */
  private final XboxController driver = new XboxController(OperatorConstants.driverPort);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton zeroGyro = 
    new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton isLocked = 
    new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton invertGyro = 
    new JoystickButton(driver, XboxController.Button.kY.value);

  /* Subsystems */
  private final Swerve swerve = new Swerve();

  /**
   * The RobotContainer class is responsible for initializing  and configuring the robot's
   * subsystems, commands, and button bindings
   */
  public RobotContainer() {

    /* Default Commands */
    swerve.setDefaultCommand(
      new SwerveDrive(
        swerve, 
        () -> driver.getRawAxis(translationAxis), 
        () -> driver.getRawAxis(strafeAxis), 
        () -> driver.getRawAxis(rotationAxis), 
        () -> true, 
        () -> isLocked.getAsBoolean()));


    configureBindings();
  }

  private void configureBindings() {
    /* Drive Buttons */
    zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));

    invertGyro.onTrue(new InstantCommand(() -> swerve.invertGyro()));

  
  }
  

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
