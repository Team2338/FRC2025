package team.gif.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class MoveLeftSlow extends Command {

    private boolean inverted;

    public MoveLeftSlow() {
        super();
        addRequirements(Robot.swerveDrive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        // move at 0.5 m/s (or 0.2 m/s if right stick is pressed)
        if( Robot.oi.driver.getHID().getRightStickButton())
            Robot.swerveDrive.drive(0.0, 0.2, 0.0);
        else
            Robot.swerveDrive.drive(0.0, 0.2, 0.0);

        if(inverted = Robot.pigeon.get360Heading() > 90 && Robot.pigeon.get360Heading() < 270)
            Robot.swerveDrive.drive(0, 0.2, 0.0);
        else
            Robot.swerveDrive.drive(0,-0.2,0);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.swerveDrive.drive(0.0, 0.0, 0.0);
    }
}
