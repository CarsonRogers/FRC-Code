// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Swerve;

public class SwerveDrive extends Command {
  private Swerve swerve;
  private DoubleSupplier m_translationSup;
  private DoubleSupplier m_strafeSup;
  private DoubleSupplier m_rotationSup;
  private BooleanSupplier m_fieldOrientated;
  private BooleanSupplier m_isLocked;

  private SlewRateLimiter m_xAxisLimiter;
  private SlewRateLimiter m_yAxisLimiter;
  
  /** Creates a new SwerveDrive. */
  public SwerveDrive(
      Swerve swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier fieldOrientated,
      BooleanSupplier isLocked) {

    this.swerve = swerve;
    addRequirements(swerve);

    m_translationSup = translationSup;
    m_strafeSup = strafeSup;
    m_rotationSup = rotationSup;
    m_fieldOrientated = fieldOrientated;
    m_isLocked = isLocked;

    m_xAxisLimiter = new SlewRateLimiter(OperatorConstants.MagnitudeSlewRate);
    m_yAxisLimiter = new SlewRateLimiter(OperatorConstants.MagnitudeSlewRate);
  }

  /**
   * 
   */
  @Override
  public void execute() {
    /* Gets Values, Applies Deadband */
    double xAxis = 
      MathUtil.applyDeadband(m_translationSup.getAsDouble(), OperatorConstants.stickDeadband);
    double yAxis = 
      MathUtil.applyDeadband(m_strafeSup.getAsDouble(), OperatorConstants.stickDeadband);
    double rAxis = 
      MathUtil.applyDeadband(m_rotationSup.getAsDouble(), OperatorConstants.stickDeadband);

    /* Squares the input */
    double xAxisSquared = xAxis * xAxis * Math.signum(xAxis);
    double yAxisSquared = yAxis * yAxis * Math.signum(yAxis);

    /* Applies Rate Limiter */
    double xAxisFiltered = m_xAxisLimiter.calculate(xAxisSquared);
    double yAxisFiltered = m_yAxisLimiter.calculate(yAxisSquared);

    /* Drive */
    swerve.drive(
      new Translation2d(-xAxisFiltered, -yAxisFiltered).times(DriveConstants.MaxSpeed),
      -rAxis * DriveConstants.MaxAngularSpeed,
      m_fieldOrientated.getAsBoolean(),
      true,
      m_isLocked.getAsBoolean());
  };

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
