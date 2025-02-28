package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class AutonShooterEmpty extends Command {

    public AutonShooterEmpty() {
        super();
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
        //don't stop the command until both sensors are false
        //return false when not empty
        return !Robot.shooter.getIndexerSensorState() && !Robot.shooter.getExitSensorState();
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
}
