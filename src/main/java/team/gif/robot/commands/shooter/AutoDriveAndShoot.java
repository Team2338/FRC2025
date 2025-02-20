package team.gif.robot.commands.shooter;

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
        if (Robot.shooter.isShooterAligned() && !hasTarget) {
            Robot.shooter.runShooterMotor();
            Robot.swerveDrive.drive(0.0, 0.0, 0.0);
            hasTarget = true;
        } else if (!hasTarget) {
                boolean inverted = moveRight;
                //If we are facing the alliance wall, invert
                inverted = Robot.pigeon.get360Heading() > 90 && Robot.pigeon.get360Heading() < 270 ? !inverted : inverted;
                double speed = Constants.Shooter.ALIGN_STRAFE_SPEED_MPS * (inverted ? -1 : 1);
                Robot.swerveDrive.drive(0, speed, 0.0);
            }
        }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return Robot.shooter.isShooterAligned();
        //TODO: This needs to be based off of the gamepiece sensors, not the ToF sensors
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("finished auto drive shoot");

        Robot.shooter.stopShooterMotor();
        Robot.swerveDrive.setDrivePace(drivePace.COAST_FR);
    }
}
