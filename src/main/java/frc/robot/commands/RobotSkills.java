package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterIntake.Intake;
import frc.robot.subsystems.ShooterIntake.Shooter;
import frc.robot.subsystems.swerve.rev.RevSwerve;

import frc.robot.States;
import frc.robot.constants.RobotSkillsConstants;

public class RobotSkills
{
    private RevSwerve s_Swerve;
    private Intake s_Intake;
    private Shooter s_Shooter;
    private RobotSkillsConstants constants = new RobotSkillsConstants();

    public RobotSkills(RevSwerve swerve, Intake intake, Shooter shooter) 
    {
        this.s_Swerve = swerve;
        this.s_Intake = intake;
        this.s_Shooter = shooter;
    }

    public Command floorAngle()
    {
        return Commands.runOnce(() -> States.armState = States.ArmStates.Floor);
    }

    public Command speakerAngle()
    {
        return Commands.runOnce(() -> States.armState = States.ArmStates.Speaker);
    }

    public Command sourceAngle()
    {
        return Commands.runOnce(() -> States.armState = States.ArmStates.Source);
    }

    public Command ampAngle()
    {
        return Commands.runOnce(() -> States.armState = States.ArmStates.Amp);
    }

    public Command shootFast()
    {
        return s_Shooter.fast().alongWith(new WaitCommand(1.5).andThen(s_Intake.fast()));
    }

    public Command shootFastAuto()
    {
        return new ParallelDeadlineGroup
        (
            new WaitCommand(2.5), 
            shootFast()
        );
    }

    public Command shootSlow()
    {
        return s_Shooter.slow().alongWith(s_Intake.slow());
    }

    public Command intakeNote()
    {
        return s_Intake.fast().until(s_Intake::hasNote);
    }

    public Command zeroGyro()
    {
        return s_Swerve.setGyroToZero();
    }

    public Command shootOnly()
    {
        return new SequentialCommandGroup
        (
            this.speakerAngle(),
            new WaitCommand(2),
            this.shootFastAuto()
        );
    }

    public Command shootCenterNotes()
    {
        return new SequentialCommandGroup(
            this.speakerAngle(),
            this.shootFastAuto(),
            this.floorAngle(),
            new WaitCommand(.5),
            new ParallelCommandGroup
            (
                this.intakeNote(),
                new AutoDriveCommand(s_Swerve, "backward", 
                    constants.backwardsRollInches, 
                    constants.backwardsRollSeconds)
            ),
            this.speakerAngle(),
            new AutoDriveCommand
            (
                s_Swerve, "forward", 
                constants.forwardRollInches, 
                constants.forwardRollSeconds
            ),
            this.shootFastAuto()
        );
    }

    public Command shootSideNote(String direction)
    {
        String outDirection = direction;
        String returnDirection = "left";
        if (direction == "left")
        {
            returnDirection = "right";
        }
        return new SequentialCommandGroup
        (
            this.floorAngle(),
            new AutoDriveCommand
            (
                s_Swerve, outDirection, 
                constants.sideRollInches, 
                constants.sideRollSeconds
            ),
            new ParallelCommandGroup
            (
                this.intakeNote(),
                new AutoDriveCommand
                (
                    s_Swerve, "backwards", 
                    constants.backwardsRollInches, 
                    constants.backwardsRollSeconds
                )
            ),
            this.speakerAngle(),
            new AutoDriveCommand
            (
                s_Swerve, returnDirection, 
                constants.sideRollInches,
                constants.sideRollSeconds
            ),
            new AutoDriveCommand
            (
                s_Swerve, "forward", 
                constants.forwardRollInches, 
                constants.forwardRollSeconds
            ),
            this.shootFastAuto()
        );
    }

    public Command threeNoteAuto()
    {
        return new SequentialCommandGroup
        (
            this.shootCenterNotes(),
            this.shootSideNote("left"),
            this.zeroGyro()
        );
    }

    public Command fourNoteAuto()
    {
        return new SequentialCommandGroup
        (
            this.shootCenterNotes(),
            this.shootSideNote("right"),
            this.shootSideNote("left"),
            this.zeroGyro()
        );
    }

    public Command shootSourceSideandRoll()
    {
        return new SequentialCommandGroup
        (
            this.shootFastAuto(),
            new AutoDriveCommand
            (
                s_Swerve, "backwards",
                constants.backwardsRollSourceInches,
                constants.backwardsRollSourceSeconds
            )
        );
    }
}
