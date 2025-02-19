package team.gif.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class ElevatorManualControl extends Command {

    public ElevatorManualControl() {
        super();
        addRequirements(Robot.elevator);
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
            percent = percent/9;
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
        //return !Robot.elevator.elevatorManualFlag;
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.elevator.setElevatorTargetPos(Robot.elevator.getPosition());
        Robot.elevator.PIDHold();
        Robot.elevator.enableLowerSoftLimit(true);
    }
}