package team.gif.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.lib.drivePace;
import team.gif.robot.Robot;

public class ShortDriveAway extends Command {
    private int counter;

    public ShortDriveAway() {
        super();
        addRequirements(Robot.swerveDrive); // uncomment
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
        Robot.swerveDrive.drive(-0.15, 0.0, 0.0);
        counter++;
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return counter >= 15;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.swerveDrive.setDrivePace(drivePace.COAST_FR);
        Robot.swerveDrive.drive(0.0,0.0,0.0);
    }
}