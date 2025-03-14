package team.gif.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.lib.RobotMode;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class ElevatorManualControl extends Command {

    public ElevatorManualControl() {
        super();
        addRequirements(Robot.elevator);
        Robot.elevator.setElevatorManualMode(true);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        double percent = -Robot.oi.aux.getLeftY();

        if (percent > Constants.Elevator.MIN_PERCENT_MANUAL && percent < Constants.Elevator.MAX_PERCENT_MANUAL){
            percent = Constants.Elevator.PID_HOLD_FF; // apply minimum FeedForward to keep the elevator from falling
        } else {
            if (percent < 0 ){
                percent = percent/5; // down (smaller = faster)
            } else {
                percent = percent/2.5; // up (smaller = faster)
            }
        }

        if (Robot.elevator.isStalled()) { // ToDo - stall feature needs testing/reworking
            percent = Constants.Elevator.PID_HOLD_FF;
        }

        Robot.elevator.move(percent);

        // Allows user to run past 0 set point if pressing the right stick
        if (Robot.oi.aux.getHID().getRightStickButton()) {
            Robot.elevator.enableLowerSoftLimit(false);
        } else {
            Robot.elevator.enableLowerSoftLimit(true);
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
        Robot.elevator.setElevatorManualMode(false);
        Robot.elevator.setElevatorTargetPos(Robot.elevator.getPosition());
        Robot.elevator.enableLowerSoftLimit(true);
    }
}