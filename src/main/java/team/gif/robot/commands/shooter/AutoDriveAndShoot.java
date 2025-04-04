package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team.gif.lib.drivePace;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.commands.drivetrain.DriveRight;
import team.gif.robot.commands.drivetrain.ShortDriveAway;
import team.gif.robot.commands.elevator.SetElevatorPosition;

public class AutoDriveAndShoot extends Command {

    private boolean hasTarget;
    private final boolean moveRight;

    /**
    * Drive right (or left) until robot finds target with both sensors,
     * then shoot the Coral (by scheduling shoot command) <br>
     * Do not shoot if the reef was never detected by both sensors. <br>
     *
     * @param moveRight set to true of the robot should move right (based on field relative)
    */
    public AutoDriveAndShoot(boolean moveRight) {
        super();
        this.moveRight = moveRight;
        addRequirements(Robot.shooter, Robot.swerveDrive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.swerveDrive.setDrivePace(drivePace.COAST_RR);
        hasTarget = Robot.shooter.isShooterAligned();
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        if (hasTarget) {
            Robot.swerveDrive.stopDrive();
            hasTarget = true;
        } else {
            boolean inverted = moveRight;
            //If we are facing the alliance wall, invert
            inverted = Robot.pigeon.get360Heading() > 90 && Robot.pigeon.get360Heading() < 270 ? !inverted : inverted;
            double speed = Constants.Shooter.ALIGN_STRAFE_SPEED_MPS * (inverted ? -1 : 1);
            Robot.swerveDrive.drive(0, speed, 0.0);

            hasTarget = Robot.shooter.isShooterAligned();
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return Robot.elevator.isReadyToShoot() && hasTarget;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.swerveDrive.stopDrive();
        Robot.swerveDrive.setDrivePace(drivePace.COAST_FR);

        // only shoot if the robot found the target during the command
        if (Robot.elevator.isReadyToShoot() && hasTarget) {
            if (!Robot.grabber.isOut()) {
                // Run the shooter using the standard shoot command and return the elevator
                // The "drive away" and "move the elevator" need to be in separate schedulers
                // or the driver won't be able to move the robot until the move elevator command
                // has completed. Need to delay the elevator until the shooter times out.
                // Effectively, this executes the drive away and moving of the elevator at the same time
                new SequentialCommandGroup(
                        new Shoot(),
                        new ShortDriveAway()
                ).schedule();
                // only return the elevator if the grabber is retracted. This is so we can shoot and then immediately
                // remove the algae, but need to strafe first
                new SequentialCommandGroup(
                        new WaitCommand(Constants.Shooter.SHOOT_CYCLES * 0.020), // scheduler runs every 20 ms
                        new SetElevatorPosition(0)
                ).schedule();
            } else {
                new SequentialCommandGroup(
                        new Shoot(),
                        new DriveRight()
                ).schedule();
            }
        }
    }
}
