package team.gif.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.lib.drivePace;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

/**
 * This command moves the robot a short distance for the purpose
 * of moving away from the Reef to bring down the elevator without
 * hitting the coral on the Reef branch
 *
 * Moves only for level 4, returns immediately for other levels
 */
public class LongDriveAway extends Command {
    private int counter;

    public LongDriveAway() {
        super();
        addRequirements(Robot.swerveDrive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //put this in initialize so that it doesn't stop the command as the elevator lowers
        Robot.swerveDrive.setDrivePace(drivePace.COAST_RR);
        counter = 0;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        // only move if elevator target is level 4
        // otherwise end the command by setting the counter very high
        Robot.swerveDrive.drive(-Constants.Shooter.LONG_DRIVE_SPEED_MPS, 0.0, 0.0);
        counter++;

    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return counter >= 0.75 * 50;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.swerveDrive.setDrivePace(drivePace.COAST_FR);
        Robot.swerveDrive.stopDrive();
    }
}