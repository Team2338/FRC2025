package team.gif.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

/**
 * This command acts as a blocking command using the distance
 * between the elevator and coral on the Reef.
 * Continues to run if the distance is less than value
 * and ends if the distance is greater than the value
 */
public class SafeToLower extends Command {

    public SafeToLower() {
        super();
        addRequirements(Robot.elevator);
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {}

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return Robot.shooter.getLeftD() > 160 && Robot.shooter.getRightD() > 160;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
}

