package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public final class ShooterIntakeConstants 
{
    // SPEEDS FOR ALL MOTORS
    // May be necessary to override these down
    // below for specific motors.
    public static final double FULL = 1.0;
    public static final double HALF = 0.5;
    public static final double QUARTER = 0.25;

    /* ARM SUBSYSTEM */
    public static final class Arm 
    {
        // ARM MOTOR IDS AND SPEEDS
        public static final int LEFT_ARM_MOTOR_ID = 8;
        public static final int RIGHT_ARM_MOTOR_ID = 11;
        public static final double FULL_SPEED = FULL;
        public static final double HALF_SPEED = HALF;
        public static final double QUARTER_SPEED = QUARTER;
        public static final int ARM_ENCODER_ID = 0;

        //ARM PHOTOEYE LIMIT SWITCH
        public static final int PHOTOEYE_DIO_ID = 1;
    
        // ARM ANGLES
        // Need to determine the starting offset angle of the
        // Rev Through Bore Encoder and set it here:
        public static final Rotation2d ARM_ENCODER_OFFSET = Rotation2d.fromDegrees(350+97);
        public static final double ARM_FLOOR_ANGLE = 90.5;
        public static final double ARM_SOURCE_ANGLE = 30.1;
        public static final double ARM_AMP_ANGLE = 1.3;
        public static final double ARM_LONGSHOT_ANGLE = 0.0;
        public static final double ARM_SPEAKER_ANGLE = 80.1;
        public static final double ARM_CLIMB_ANGLE = 70.0;

        // ARM PID 
        public static final int ARM_CURRENT_LIMIT = 50;
        public static final double ARM_UPDATE_OUTPUT = .02;
        public static final double ARM_MAX_VELOCITY = 1.75;
        public static final double ARM_MAX_ACCELERATION = .75;
        public static final double ARM_P = .2;
        public static final double ARM_I = 0.0;
        public static final double ARM_D = 0.7;
        public static final double ARM_S = 1.1;
        public static final double ARM_G = 1.2;
        public static final double ARM_V = 1.3;
    }

    /* SHOOTER PORTION OF SHOOTERINTAKE SUBSYSTEM */
    public static final class Shooter 
    {
        public static final int LEFT_SHOOTER_MOTOR_ID = 10;
        public static final int RIGHT_SHOOTER_MOTOR_ID = 9;
        public static final double FULL_SPEED = FULL * .7;
        public static final double HALF_SPEED = HALF * .3;
        public static final double QUARTER_SPEED = QUARTER;
    }

    /* INTAKE PORTION OF SHOOTERINTAKE SUBSYSTEMS */
    public static final class Intake 
    {
        public static final int INTAKE_MOTOR_ID = 0;
        public static final int PHOTOEYE_DIO_ID = 2;
        public static final double FULL_SPEED = .5;
        public static final double HALF_SPEED = .4;
        public static final double QUARTER_SPEED = .07;
    }
}
