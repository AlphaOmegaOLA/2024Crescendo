// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.States;
import frc.robot.constants.ShooterIntakeConstants;

public class PIDArmCommand extends Command 
{
  /** Creates a new PIDArmCommand. */
  private frc.robot.subsystems.ShooterIntake.PIDArm PIDArm;

 private PIDController armController;
  public PIDArmCommand(frc.robot.subsystems.ShooterIntake.PIDArm PIDArm) 
  {
      // Use addRequirements() here to declare subsystem dependencies.
      this.PIDArm = PIDArm;
      addRequirements(PIDArm);

      armController = new PIDController(Constants.articulation.armP, Constants.articulation.armI, Constants.articulation.armD);
      armController.setTolerance(0.01);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

    switch(States.armState)
    {
        case standard:
          PIDArm.setAngle(PIDArm.getAngle());
          break;
        case Source:
            PIDArm.setAngle(ShooterIntakeConstants.Arm.ARM_SOURCE_ANGLE);
            break;
        case Amp:
            PIDArm.setAngle(ShooterIntakeConstants.Arm.ARM_AMP_ANGLE);
            break;
        case Speaker:
            PIDArm.setAngle(ShooterIntakeConstants.Arm.ARM_SPEAKER_ANGLE);
            break;
        case Floor:
            PIDArm.setAngle(ShooterIntakeConstants.Arm.ARM_FLOOR_ANGLE);
            break;
    }

    PIDArm.runArm(armController.calculate(PIDArm.getAngle(), PIDArm.setpoint));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
