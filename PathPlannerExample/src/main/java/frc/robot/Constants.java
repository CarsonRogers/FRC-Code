package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {

    /* Voltage Compensation */
    public static final int voltageComp = 12;

    /* Constants related to operator controls */
    public static class OperatorConstants{
        public static double stickDeadband = 0.15;
        
        public static int driverPort = 0;

        public static final double MagnitudeSlewRate = 2.0;
    }

    /* Constants related to the swerve modules */
    public static class ModuleConstants{

        /* Front Left Module */
        public static final class ModFL{
            public static final String name = "Front Left Module";
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int encoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(123.9);
        }

        /* Front Right Module */
        public static final class ModFR{
            public static final String name = "Front Right Module";
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int encoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(69.8);
        }

        /* Back Left Module */
        public static final class ModBL{
            public static final String name = "Back Left Module";
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 3;
            public static final int encoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(70.2);
        }

        /* Back Right Module */
        public static final class ModBR{
            public static final String name = "Back Right Module";
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int encoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(95.9);
        }
    }

    /* Constants related to swerve drive */
    public static class DriveConstants{

        public static final int GyroID = 0;
        public static final boolean GyroInvert = false;

        public static final double TrackWidth = Units.inchesToMeters(20.75);
        public static final double WheelBase = Units.inchesToMeters(20.75);
        public static final double WheelDiameter = Units.inchesToMeters(4.0);
        public static final double WheelCircumference = WheelDiameter * Math.PI;

        public static final double CenterToWheel = 
            Math.sqrt(Math.pow(WheelBase / 2.0, 2) + Math.pow(TrackWidth / 2.0, 2));

        public static final Translation2d[] moduleTranslations =
            new Translation2d[] {
                new Translation2d(WheelBase / 2.0, TrackWidth / 2.0),
                new Translation2d(WheelBase / 2.0, -TrackWidth / 2.0),
                new Translation2d(-WheelBase / 2.0, TrackWidth / 2.0),
                new Translation2d(-WheelBase / 2.0, -TrackWidth / 2.0)
            };
        
        public static final SwerveDriveKinematics swerveKinematics = 
            new SwerveDriveKinematics(moduleTranslations);


        /* Motor Gear Ratios */
        // Found depending on the swerve pod models, this bot is the SDS MK4i L2
        public static final double DriveMotorRatio = 6.75 / 1.0;
        public static final double TurnMotorRatio = 150.0 / 7.0 / 1.0;

        /* Motor Conversion Factors */
        public static final double DriveConversionFactor = 
            (WheelCircumference / DriveMotorRatio);
        public static final double AngleConversionFactor = 
            360 / TurnMotorRatio;

        public static final double DriveVelocityConversionFactor = 
            DriveConversionFactor / 60;
        public static final double AngleVelocityConversionFactor = 
            (2 * Math.PI) / TurnMotorRatio / 60;

        /* Speed */
        public static final double MaxSpeed = 4;
        public static final double MaxAngularSpeed = Math.PI;

        /* Drive Feed Forward */
        public static final double DriveKS = 0.0;
        public static final double DriveKV = 0.0;
        public static final double DriveKA = 0.0;

        /* Drive PID */
        public static final double DriveP = 0.04;
        public static final double DriveI = 0.00;
        public static final double DriveD = 0.00;

        public static final double DriveF = 
            1 / ((NeoMotorConstants.FreeSpeedRPM / 60) * WheelCircumference) / DriveMotorRatio;

        /* Turn PID */
        public static final double TurnP = 0.01;
        public static final double TurnI = 0.0;
        public static final double TurnD = 0.0;

        public static final double TurnF = 0.0;

        /* Drive Motor */
        public static final int DriveContinousCurrentLimit = 40;
        public static final boolean DriveInvert = false;
        public static final IdleMode DriveIdleMode = IdleMode.kBrake;

        /* Angle Motor */
        public static final int AngleContinuousCurrentLimit = 20;
        public static final boolean AngleInvert = true;
        public static final IdleMode AngleIdleMode = IdleMode.kCoast;

        public static final HolonomicPathFollowerConfig SwerveHolonomicPathFollowerConfig = 
            new HolonomicPathFollowerConfig(
                new PIDConstants(
                    AutoConstants.AutoDriveD, AutoConstants.AutoDriveI, AutoConstants.AutoDriveD),
                new PIDConstants(
                    AutoConstants.AutoTurnP, AutoConstants.AutoTurnI, AutoConstants.AutoTurnD),
                AutoConstants.MaxSpeed,
                CenterToWheel,
                new ReplanningConfig(
                    true, true)
            );
    }

    public static class AutoConstants{

        /* Speeds */
        public static final double MaxSpeed = 4.0;

        /* PID Constants */
        public static final double AutoTurnP = 0.0;
        public static final double AutoTurnI = 0.0;
        public static final double AutoTurnD = 0.0;

        public static final double AutoDriveP = 0.0;
        public static final double AutoDriveI = 0.0;
        public static final double AutoDriveD = 0.0;
    }

    public static class NeoMotorConstants{
        public static final double FreeSpeedRPM = 5676.0;
    }
}
