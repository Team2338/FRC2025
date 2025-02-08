package team.gif.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.lib.drivePace;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class AutoDriveAndShoot extends Command {

    private boolean hasTarget;
    private final boolean moveRight;

    public AutoDriveAndShoot(boolean moveRight) {
        super();
        this.moveRight = moveRight;
        addRequirements(Robot.shooter, Robot.swerveDrive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        hasTarget = false;
        Robot.swerveDrive.setDrivePace(drivePace.COAST_RR);
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        if (Robot.shooter.isFireReady() && !hasTarget) {
            Robot.shooter.runShooterMotor(Constants.Shooter.SPEED_PERCENT);
            Robot.swerveDrive.drive(0.0, 0.0, 0.0);
            hasTarget = true;
        } else if (!hasTarget) {
                boolean inverted = moveRight;
                //If we are facing the alliance wall, invert
                inverted = Robot.pigeon.get360Heading() > 90 && Robot.pigeon.get360Heading() < 270 ? !inverted : inverted;
                double speed = Constants.Shooter.ALIGN_SPEED_MPS * (inverted ? -1 : 1);
                Robot.swerveDrive.drive(speed, 0.0, 0.0);
            }
        }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.shooter.runShooterMotor(0);
        Robot.swerveDrive.setDrivePace(drivePace.COAST_FR);
    }
}
