package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team.gif.robot.commands.drivetrain.ShortDriveAway;
import team.gif.robot.commands.elevator.SetElevatorPosition;

public class AutonShootReset extends SequentialCommandGroup {
    public AutonShootReset() {
        addCommands(
//                new MoveModulesIn(),
                new AutonAutoTarget(),
                new ParallelRaceGroup(
                        new AutonHasNoTarget(),
                        new Shoot()
                ),
                new ParallelCommandGroup(
                        new ShortDriveAway(),
                        new SetElevatorPosition(0)
                )
        );
    }
}
