package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.swerve.rev.RevSwerve;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class MoveBackwardsCommand extends Command 
{
    private final RevSwerve swerveDrive;
    private final double distanceMeters;
    private Pose2d initialPose;
    private boolean isFinished;

    public MoveBackwardsCommand(RevSwerve swerveDrive, double distanceFeet) 
    {
        this.swerveDrive = swerveDrive;
        this.distanceMeters = distanceFeet * 0.3048; // Convert feet to meters

        // This method binds the command to the subsystem dependencies.
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() 
    {
        initialPose = swerveDrive.getPose();
        isFinished = false;
    }

    @Override
    public void execute() 
    {
        double currentDistance = Math.abs(swerveDrive.getPose().getX() - initialPose.getX());
        if (currentDistance < distanceMeters) 
        {
            swerveDrive.drive(new Translation2d(-0.5, 0), 0, true, false);
        } 
        else 
        {
            isFinished = true;
        }
    }

    @Override
    public void end(boolean interrupted) 
    {
        swerveDrive.drive(new Translation2d(0, 0), 0, true, false);
    }

    @Override
    public boolean isFinished() 
    {
        return isFinished;
    }

    // This ensures that the command requires the swerve drive subsystem
    // Replace `Subsystem` with the specific type if using a type stricter than Subsystem.
    private void addRequirements(Subsystem subsystem) 
    {
        // Method stub for dependency binding if required by your command structure.
        // The actual implementation should bind this command to the swerveDrive subsystem.
    }
}
