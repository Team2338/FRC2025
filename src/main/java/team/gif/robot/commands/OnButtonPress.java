package team.gif.robot.commands;

import com.fasterxml.jackson.databind.introspect.ConcreteBeanPropertyBase;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class OnButtonPress extends Command {

    public OnButtonPress() {
        super();
        addRequirements(Robot.shooter); // uncomment
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        if (Robot.shooter.isFireReady() == true) {
            Robot.shooter.moveMotor(Constants.SHOOTER_SPEED_PERCENT);
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) { Robot.shooter.moveFromShuffleboard(); }
}
