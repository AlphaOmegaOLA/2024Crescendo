package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public final class ShooterIntakeConstants 
{
    public static final double FULL = 1.0;
    public static final double HALF = 0.5;
    public static final double QUARTER = 0.25;

    public static final class Arm 
    {
        public static final int LEFT_ARM_MOTOR_ID = 23;
        public static final int RIGHT_ARM_MOTOR_ID = 24;
        public static final double FULL_SPEED = FULL;
        public static final double HALF_SPEED = HALF;
        public static final double QUARTER_SPEED = QUARTER;
        public static final int ARM_ENCODER_ID = 0;
        // Need to determine the starting offset angle of the
        // Rev Through Bore Encoder and set it here:
        public static final Rotation2d ARM_ENCODER_OFFSET = Rotation2d.fromDegrees(0);
        public static final int ARM_CURRENT_LIMIT = 40;
    }

    public static final class Shooter 
    {
        public static final int LEFT_SHOOTER_MOTOR_ID = 20;
        public static final int RIGHT_SHOOTER_MOTOR_ID = 21;
        public static final double FULL_SPEED = FULL;
        public static final double HALF_SPEED = HALF;
        public static final double QUARTER_SPEED = QUARTER;
    }

    public static final class Intake 
    {
        public static final int INTAKE_MOTOR_ID = 22;
        public static final double FULL_SPEED = FULL;
        public static final double HALF_SPEED = HALF;
        public static final double QUARTER_SPEED = QUARTER;
    }
}
