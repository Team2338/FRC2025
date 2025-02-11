package team.gif.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
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
        double percent = -Robot.oi.aux.getRightY();

        if (percent > -0.15 && percent < 0.15) {
            percent = 0.15; // apply minimum FeedForward to keep the elevator from falling (0.10 is max before elevator begins to move)
        }

        Robot.elevator.move(percent);

        // Allows user to run past 0 setpoint if pressing the right stick
        if (Robot.oi.aux.getHID().getRightStickButton()) {
            Robot.elevator.enableLowerSoftLimit(false);
        } else {
            Robot.elevator.enableLowerSoftLimit(true);
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return !Robot.elevator.elevatorManualFlag;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.elevator.setElevatorTargetPos(Robot.elevator.getPosition());
        Robot.elevator.PIDHold();
        Robot.elevator.enableLowerSoftLimit(true);
    }
}