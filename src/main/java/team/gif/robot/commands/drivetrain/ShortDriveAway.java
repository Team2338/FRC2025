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
public class ShortDriveAway extends Command {
    private int counter;

    public ShortDriveAway() {
        super();
        addRequirements(Robot.swerveDrive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.swerveDrive.setDrivePace(drivePace.COAST_RR);
        counter = 0;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        // only move if elevator target is level 4
        // otherwise end the command by setting the counter very high
        if (Robot.elevator.getPosition() > Constants.Elevator.LEVEL_4_POSITION - 10) {
            Robot.swerveDrive.drive(-0.30, 0.0, 0.0);
            counter++;
        } else {
            counter = 5000; // set counter to a very large number (5 seconds) so command will quit immediately
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return counter >= 10;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.swerveDrive.setDrivePace(drivePace.COAST_FR);
//        Robot.swerveDrive.drive(0.0,0.0,0.0);
        System.out.println("Ending short drive");
    }
}