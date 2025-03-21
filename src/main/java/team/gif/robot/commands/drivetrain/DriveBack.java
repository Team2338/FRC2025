package team.gif.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class DriveBack extends Command {
    public int counter;
    public double rLInitPosition, rRInitPosition;

    public DriveBack() {
        super();
        addRequirements(Robot.swerveDrive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        counter = 0;
        rLInitPosition = Robot.swerveDrive.rL.getPosition().distanceMeters;
        rRInitPosition = Robot.swerveDrive.rR.getPosition().distanceMeters;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        counter++;
        Robot.swerveDrive.drive(1, 0, 0);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        double rLPosition = Robot.swerveDrive.rL.getPosition().distanceMeters;
        double rRPosition = Robot.swerveDrive.rR.getPosition().distanceMeters;
        return counter > 0.5 * 50 ||
                (Math.abs(rLPosition - rLInitPosition) > Units.inchesToMeters(3) &&
                        Math.abs(rRPosition - rRInitPosition) > Units.inchesToMeters(3));
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.swerveDrive.stopDrive();
    }
}
