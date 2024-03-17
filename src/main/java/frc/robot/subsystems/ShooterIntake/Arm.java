package frc.robot.subsystems.ShooterIntake;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.ShooterIntakeConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;


import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends SubsystemBase 
{
    //private DigitalInput photoEye = new DigitalInput(ShooterIntakeConstants.Arm.PHOTOEYE_DIO_ID);
    private CANSparkMax leftArmMotor;
    private CANSparkMax rightArmMotor;
    private RevThroughBoreEncoder armEncoder;
    private static double updateOutput;
    private static double maxVelocity;
    private static double maxAcceleration;
    private static double proportional;
    private static double integral;
    private static double differential;
    private static double staticFriction;
    private static double gravityGain;
    private static double velocityGain;
    private final TrapezoidProfile.Constraints constraints;
    private final ProfiledPIDController controller;
    private final ElevatorFeedforward feedForward;
    private double voltage;
    private RelativeEncoder leftEncoder;
    private double currentAngle;
    private double progress;
    private double rotations;
    private boolean goalReached;

    public Arm()
    {
        leftArmMotor = new CANSparkMax(ShooterIntakeConstants.Arm.LEFT_ARM_MOTOR_ID, MotorType.kBrushless);
        leftArmMotor.setIdleMode(IdleMode.kBrake);
        leftArmMotor.setInverted(true);
        rightArmMotor = new CANSparkMax(ShooterIntakeConstants.Arm.RIGHT_ARM_MOTOR_ID, MotorType.kBrushless);
        rightArmMotor.setIdleMode(IdleMode.kBrake);
        //rightArmMotor.setInverted(true);
        armEncoder = new RevThroughBoreEncoder(ShooterIntakeConstants.Arm.ARM_ENCODER_ID);
        armEncoder.setOffset(ShooterIntakeConstants.Arm.ARM_ENCODER_OFFSET);
        //armEncoder.setInverted(true);
        updateOutput = ShooterIntakeConstants.Arm.ARM_UPDATE_OUTPUT;
        maxVelocity = ShooterIntakeConstants.Arm.ARM_MAX_VELOCITY;
        maxAcceleration = ShooterIntakeConstants.Arm.ARM_MAX_ACCELERATION;
        proportional = ShooterIntakeConstants.Arm.ARM_P;
        integral = ShooterIntakeConstants.Arm.ARM_I;
        differential = ShooterIntakeConstants.Arm.ARM_D;
        staticFriction = ShooterIntakeConstants.Arm.ARM_S;
        gravityGain = ShooterIntakeConstants.Arm.ARM_G;
        velocityGain = ShooterIntakeConstants.Arm.ARM_V;
        constraints =  new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
        controller = new ProfiledPIDController(proportional, integral, differential, constraints, updateOutput);
        feedForward = new ElevatorFeedforward(staticFriction, gravityGain, velocityGain);
        leftEncoder = leftArmMotor.getEncoder();
        goalReached = false;
    }

    public void setSpeaker()
    {
        while (armEncoder.getAngle().getDegrees() < ShooterIntakeConstants.Arm.ARM_SPEAKER_ANGLE)
        {
            goalReached = false;   
            leftArmMotor.set(.5);
            rightArmMotor.set(.5);
        }
        leftEncoder.setPosition(0);
        leftArmMotor.set(0);
        rightArmMotor.set(0);
        goalReached = true;
    }

    public void setSource()
    {
        while (armEncoder.getAngle().getDegrees() > ShooterIntakeConstants.Arm.ARM_SOURCE_ANGLE)
        {
            goalReached = false;
            leftArmMotor.set(-.5);
            rightArmMotor.set(-.5);
        }
        leftEncoder.setPosition(0);
        leftArmMotor.set(0);
        rightArmMotor.set(0);
        goalReached = true;
    }

    private void setAngle(double angle)
    {
        //SmartDashboard.putNumber("Goal", angle);
        leftEncoder.setPosition(0);
        System.out.println("IN SET ANGLE");
        System.out.println("GOAL:" + angle);
        System.out.println("AMP ANGLE:" + ShooterIntakeConstants.Arm.ARM_AMP_ANGLE);
        System.out.println("FLOOR ANGLE:" + ShooterIntakeConstants.Arm.ARM_FLOOR_ANGLE);
        goalReached = false;
        // Arm is currently less than the desired angle
        if (angle > ShooterIntakeConstants.Arm.ARM_AMP_ANGLE)
        {
            System.out.println(angle + " is > " + ShooterIntakeConstants.Arm.ARM_AMP_ANGLE);
            System.out.println("LOWERING ARM");
            progress = angle / ShooterIntakeConstants.Arm.ARM_FLOOR_ANGLE;
            rotations = 161 * progress;
            leftEncoder.setPosition(0);
            if (armEncoder.getAngle().getDegrees() < angle && leftEncoder.getPosition() <= rotations)
            {
                goalReached = false;
                System.out.println("IN LOWER MODE");
                leftArmMotor.set(ShooterIntakeConstants.Arm.HALF_SPEED);
                rightArmMotor.set(ShooterIntakeConstants.Arm.HALF_SPEED);
                //SmartDashboard.putNumber("LEFT ENCODER", leftEncoder.getPosition());
            }
            else
            {
                System.out.println("STOPPED LOWERING");
                leftEncoder.setPosition(0);
                leftArmMotor.set(0);
                rightArmMotor.set(0);
                goalReached = true;
            }
        }
        // Arm is currently greater than the desired angle
        else if (angle < ShooterIntakeConstants.Arm.ARM_FLOOR_ANGLE)
        {
            System.out.println(angle + " is < " + ShooterIntakeConstants.Arm.ARM_FLOOR_ANGLE);
            System.out.println("RAISING ARM");
            goalReached = false;
            progress = angle / ShooterIntakeConstants.Arm.ARM_AMP_ANGLE;
            rotations = -161 * progress;
            leftEncoder.setPosition(0);
            if (armEncoder.getAngle().getDegrees() > angle && leftEncoder.getPosition() >= rotations)
            {
                System.out.println("IN RAISE MODE");
                goalReached = false;
                leftArmMotor.set(-ShooterIntakeConstants.Arm.HALF_SPEED);
                rightArmMotor.set(-ShooterIntakeConstants.Arm.HALF_SPEED);
                //SmartDashboard.putNumber("LEFT ENCODER", leftEncoder.getPosition());
            }
            else
            {
                System.out.println("STOPPED RAISING");
                leftArmMotor.set(0);
                rightArmMotor.set(0);            
                leftEncoder.setPosition(0);
                goalReached = true;
            }
        }
        else
        {
            System.out.println("NO MORE ARM ACTIONS");
            leftArmMotor.set(0);
            rightArmMotor.set(0);
            leftEncoder.setPosition(0);
            goalReached = true;
        }
    }

    public void intakeFloor()
    {
        setAngle(ShooterIntakeConstants.Arm.ARM_FLOOR_ANGLE);
    }

    public void intakeSource()
    {
        setAngle(ShooterIntakeConstants.Arm.ARM_SOURCE_ANGLE);
    }

    public void shootAmp()
    {
        setAngle(ShooterIntakeConstants.Arm.ARM_AMP_ANGLE);
    }
    
    public void shootLong()
    {
        setAngle(ShooterIntakeConstants.Arm.ARM_LONGSHOT_ANGLE);
    }

    public void shootSpeaker()
    {
        setAngle(ShooterIntakeConstants.Arm.ARM_SPEAKER_ANGLE);
    }

    public void climb()
    {
        setAngle(ShooterIntakeConstants.Arm.ARM_CLIMB_ANGLE);
    }

    public boolean tooFar()
    {
        // Can invert this with ! if wiring is backwards
        //return photoEye.get();
        return false;
    }

    public double getAngle()
    {
        return armEncoder.getAngle().getDegrees();
    }

    public boolean reachedGoal()
    {
        return goalReached;
    }

    public void stopMotors()
    {
        leftArmMotor.set(0.0);
        rightArmMotor.set(0.0);
    }

    public void resetEncoders()
    {
        //These are used as a backup to the Rev Through bore encoder
        leftEncoder.setPosition(0.0);
    }

    public void periodic()
    {
        SmartDashboard.putNumber("ARM ANGLE", armEncoder.getAngle().getDegrees());
        //SmartDashboard.putNumber("ARM OFFSET", armEncoder.getOffset().getDegrees());
        //SmartDashboard.putNumber("LEFT ENCODER", leftEncoder.getPosition());
        //SmartDashboard.putNumber("CPR", leftEncoder.getCountsPerRevolution());
    }
}

