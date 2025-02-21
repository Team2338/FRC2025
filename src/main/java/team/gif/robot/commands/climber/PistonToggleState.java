package team.gif.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

/**
 * Changes the state of the piston between Out and In
 */
public class PistonToggleState extends Command {

    public PistonToggleState() {
        super();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (Robot.climber.getPistonStateOut()) {
            Robot.climber.setPistonIn();
        } else {
            Robot.climber.setPistonOut();
        }
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {}

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return true;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
}