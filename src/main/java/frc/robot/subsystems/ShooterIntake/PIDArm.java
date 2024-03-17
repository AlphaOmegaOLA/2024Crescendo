// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ShooterIntake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterIntakeConstants;

public class PIDArm extends SubsystemBase 
{
  /** Creates a new PIDArm. */
  private CANSparkMax leftArmMotor;
  private CANSparkMax rightArmMotor;
  private RevThroughBoreEncoder armEncoder;
  public double setpoint = 0;

  public PIDArm() 
  {
        leftArmMotor = new CANSparkMax(ShooterIntakeConstants.Arm.LEFT_ARM_MOTOR_ID, MotorType.kBrushless);
        leftArmMotor.setIdleMode(IdleMode.kBrake);
        leftArmMotor.setInverted(true);
        rightArmMotor = new CANSparkMax(ShooterIntakeConstants.Arm.RIGHT_ARM_MOTOR_ID, MotorType.kBrushless);
        rightArmMotor.setIdleMode(IdleMode.kBrake);
        armEncoder = new RevThroughBoreEncoder(ShooterIntakeConstants.Arm.ARM_ENCODER_ID);
        armEncoder.setOffset(ShooterIntakeConstants.Arm.ARM_ENCODER_OFFSET);
  }

  public void setAngle(double angle)
  {
    setpoint = angle;
  }

 public void runArm(double input)
 {
  leftArmMotor.set(input);
  rightArmMotor.set(input);
 }

  public double getAngle() 
  {
    return armEncoder.getAngle().getDegrees();
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ARM ANGLE", armEncoder.getAngle().getDegrees());
  }
}
