package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

public class Constants {

    /* Voltage Compensation */
    public static final int voltageComp = 12;

    /* Constants related to operator controls */
    public static class OperatorConstants{
        public static int operatorPort = 1;
    }

    public static class IntakeConstants{
        /* Motor IDs */
        public static int intakeID = 1;

        /* Intake Speeds */
        public static double intakeSpeed = 0.3;
        public static double extakeSpeed = -0.2;

        /* Motor Inverts */
        public static boolean invertIntake = false;

        /* Motor Idle Mode */
        public static IdleMode intakeIdleMode = IdleMode.kCoast;
    }

    public static class ShooterConstants{
        /* Motor IDs */
        public static int leftShooterID = 2;
        public static int rightShooterID = 3;
        public static int auxLeftShooterID = 4;
        public static int auxRightShooterID = 5;

        /* Shooter Speeds */
        public static double shootSpeed = 0.4;
        public static double auxShootSpeed = 0.6;

        /* Motor Inverts */
        public static boolean invertRightShooter = false;
        public static boolean invertLeftShooter = true;
        public static boolean invertAuxRightShooter = false;
        public static boolean invertAuxLeftShooter = true;

        /* Motor Idle Mode */
        public static IdleMode leftShooterIdleMode = IdleMode.kCoast;
        public static IdleMode rightShooterIdleMode = IdleMode.kCoast;
        public static IdleMode auxLeftShooterIdleMode = IdleMode.kCoast;
        public static IdleMode auxRightShooterIdleMode = IdleMode.kCoast;
    }

    public static class FeederConstants{
        /* Motor IDs */
        public static int feederMotorID = 6;

        /* Feeder Speeds */
        public static double feedSpeed = 0.4;

        /* Motor Inverts */
        public static boolean invertFeeder = false;

        /* Motor Idle Mode */
        public static IdleMode feederIdleMode = IdleMode.kCoast;
    }

    public static class IntakeShooterConstants{
        /* Motor IDs */

        /* Shooter/Intake Speeds */

        /* Motor Inverts */

        /* Motor Idle Mode */

        /* Current Limit */
    }
}
