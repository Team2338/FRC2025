package team.gif.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

import static team.gif.robot.Robot.oi;

public class Rumble extends Command {
    private int rumbleCounter;

    public Rumble() {
        super();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        rumbleCounter = 0;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        rumbleCounter++;

        if (!DriverStation.isAutonomous()) {
            oi.setRumble(true);
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return rumbleCounter > 0.3*50;  // 50 cycles in 1 second
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        oi.setRumble(false);
    }
}
