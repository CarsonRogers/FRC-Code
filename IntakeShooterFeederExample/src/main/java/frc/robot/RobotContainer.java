// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {

  /* Controllers */
  private final XboxController operator = new XboxController(OperatorConstants.operatorPort);


  /* Operator Buttons */
  private final JoystickButton intakeButton = new JoystickButton(operator, XboxController.Button.kA.value);
  private final JoystickButton shootButton = new JoystickButton(operator, XboxController.Button.kX.value);
  private final JoystickButton feedButton = new JoystickButton(operator, XboxController.Button.kB.value);

  /* Subsystems */
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  private final Feeder feeder = new Feeder();

  /**
   * The RobotContainer class is responsible for initializing  and configuring the robot's
   * subsystems, commands, and button bindings. 
   * 
   * Generally, most subsystems will have a stop command that is set to the default command
   * so that if anything goes wrong, the robot will stop by default
   */
  public RobotContainer() {

    /* Default Commands */
    intake.setDefaultCommand(
      new RunCommand(
        () -> intake.stop(), 
        intake)
    );

    shooter.setDefaultCommand(
      new RunCommand(
        () -> shooter.stop(), 
        shooter)
    );

    feeder.setDefaultCommand(
      new RunCommand(
        () -> feeder.stop(), 
        feeder)
    );


    configureBindings();
  }

  private void configureBindings() {
    /* Drive Buttons */
    intakeButton.whileTrue(new RunCommand(() -> intake.intake(), intake));

    shootButton.whileTrue(new RunCommand(() -> shooter.shoot(), shooter));

    feedButton.whileTrue(new RunCommand(() -> feeder.feed(), feeder));
  }
  
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  /**
   * The printValues method is not a method originally found inside RobotContainer, but it is 
   * usefel to put everything that is getting printed to SmartDashboard into one method
   * so that it is easier to track what all is getting outputted.
   * 
   * This method will need called in the Robot.java file under the robotPeriodic() method
   */
  public void printValues(){
    SmartDashboard.putNumber("Main Shooter RPM", shooter.getRPM());
    SmartDashboard.putNumber("Aux Shooter RPM", shooter.getAuxRPM());
  }
}
