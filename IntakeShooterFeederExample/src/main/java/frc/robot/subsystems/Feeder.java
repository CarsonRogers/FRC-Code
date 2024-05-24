// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FeederConstants;

public class Feeder extends SubsystemBase {
  /** Creates a new Feeder. */

  CANSparkMax feederMotor;

  /**
   * Creates a basic one motor feeder system very similar to the basic intake. 
   * 
   * IF the feeder needs to be two motors, it is as simple as adding an additional motor and setting it
   * inverted to the current one. Make sure to either set the motor in all the different methods that make changes
   * to the motors. 
   */
  public Feeder() {
    feederMotor = new CANSparkMax(FeederConstants.feederMotorID, MotorType.kBrushless);
    configFeedMotor();
  }

  /**
   * Sets the feeder motor speed to a initialized constant by
   * calling the feed method with a parameter of that given speed
   */
  public void feed(){
    feed(FeederConstants.feedSpeed);
  }

  /**
   * Sets the feeder motor to a certian percentage output
   * @param speed the percentage output of the motor form 0 ot 1
   */
  public void feed(double speed){
    feederMotor.set(speed);
  }
  
  /**
   * Stops the feeder motor by setting the output to zero
   */
  public void stop(){
    feederMotor.set(0);
  }

  /**
   * Configures the feeder motor by restoring factory defaults then
   * applying any motor configurations as initialized in Constants
   */
  private void configFeedMotor(){
    feederMotor.restoreFactoryDefaults();
    feederMotor.enableVoltageCompensation(Constants.voltageComp);
    feederMotor.setInverted(FeederConstants.invertFeeder);
    feederMotor.setIdleMode(FeederConstants.feederIdleMode);
  }
}
