package team.gif.robot.commands.toggleManualControl;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;
//import team.gif.robot.commands.climber.ClimberManualControl;
//import team.gif.robot.commands.elevator.ElevatorManualControl;

public class ToggleManualControl extends Command {

    public ToggleManualControl() {
        super();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.enableRobotModeManual();

        // enable manual mode for the elevator
//-        new ElevatorManualControl().schedule();
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {}

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.enableRobotModeStandardOp();
    }
}
