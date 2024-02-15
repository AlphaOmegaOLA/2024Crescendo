package frc.robot.constants;

public final class ShooterIntakeConstants 
{
    public static final double full = 1.0;
    public static final double half = 0.5;
    public static final double quarter = 0.25;

    public static final class Arm
    {
        public static final int leftArmMotorID = 23;
        public static final int rightArmMotorID = 24;
        public static final double fullSpeed = full;
        public static final double halfSpeed = half;
        public static final double quarterSpeed = quarter;

    }
    
    public static final class Shooter
    {
        public static final int leftShooterMotorID = 20;
        public static final int rightShooterMotorID = 21;
        public static final double fullSpeed = full;
        public static final double halfSpeed = half;
        public static final double quarterSpeed = quarter;
    }  

    public static final class Intake
    {
        public static final int intakeMotorID = 22;
        public static final double fullSpeed = full;
        public static final double halfSpeed = half;
        public static final double quarterSpeed = quarter;
    }
}
