package team.gif.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.lib.RobotMode;
import team.gif.robot.Robot;

public class ClimberManualControl extends Command {

    public ClimberManualControl() {
        super();
        addRequirements(Robot.climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        double percent = -Robot.oi.aux.getRightY();

        if (Math.abs(percent) > 0.08) { // dead band around joystick
            Robot.climber.move(percent);
        } else {
            Robot.climber.move(0);
        }

        // Allows user to run past 0 set point if pressing the right stick
        if (Robot.oi.aux.getHID().getLeftStickButton()) {
            Robot.climber.enableSoftLimit(false);
        } else {
            Robot.climber.enableSoftLimit(true);
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        // end manual mode when operator switches back to standard mode
        return Robot.getRobotMode() == RobotMode.STANDARD_OP;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.climber.enableSoftLimit(true);
    }
}