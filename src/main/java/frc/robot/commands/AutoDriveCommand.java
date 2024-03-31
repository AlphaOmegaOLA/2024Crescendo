package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.rev.RevSwerve;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.swerve.rev.RevSwerveConfig;
import edu.wpi.first.wpilibj.Timer;

public class AutoDriveCommand extends CommandBase {
    private final RevSwerve swerveSubsystem;
    private final Translation2d direction;
    private final double speed;
    private final double time;
    private final Timer timer = new Timer();

    public AutoDriveCommand(RevSwerve swerve, String directionStr, double distanceInches, double timeSeconds) {
        this.swerveSubsystem = swerve;
        this.time = timeSeconds;

        // Convert distance from inches to meters and calculate speed
        double distanceMeters = distanceInches * 0.0254;
        this.speed = distanceMeters / timeSeconds;

        // Determine direction
        switch (directionStr.toLowerCase()) {
            case "forward":
                this.direction = new Translation2d(speed, 0);
                break;
            case "backward":
                this.direction = new Translation2d(-speed, 0);
                break;
            case "right":
                this.direction = new Translation2d(0, -speed);
                break;
            case "left":
            default:
                this.direction = new Translation2d(0, speed);
                break;
        }

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        // Adjusting the speed and rotation based on the RevSwerveConfig and current drive state
        Translation2d adjustedDirection = direction.times(RevSwerveConfig.maxSpeed);
        swerveSubsystem.drive(adjustedDirection, 0, false, true);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(new Translation2d(0, 0), 0, false, true);
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= time;
    }
}
