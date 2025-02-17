package team.gif.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class TestSwerve extends Command {

    private double counter = 0;

    public TestSwerve() {
        super();
        addRequirements(Robot.swerveDrive);
        //addRequirements(Robot.climber); // uncomment
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        counter = 0;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        counter++;
        Robot.swerveDrive.setMaxDrive();
        Robot.swerveDrive.fL.turnHoldZero();
        Robot.swerveDrive.fR.turnHoldZero();
        Robot.swerveDrive.rL.turnHoldZero();
        Robot.swerveDrive.rR.turnHoldZero();
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return counter >= 50 * 1.5;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.swerveDrive.stopModules();
    }
}
