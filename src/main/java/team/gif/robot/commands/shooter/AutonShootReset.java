package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team.gif.robot.commands.drivetrain.ShortDriveAway;
import team.gif.robot.commands.drivetrain.StopModules;
import team.gif.robot.commands.elevator.SetElevatorPosition;

public class AutonShootReset extends SequentialCommandGroup {
    public AutonShootReset() {
        addCommands(
//                new MoveModulesIn(),
                new AutonStrafeToTarget(),
                new ParallelRaceGroup( // ends when any command ends // if robot does not have a target, don't shoot
                        new Shoot(),
                        new AutonHasNoTarget(),
                        new StopModules()
                ),
                new ParallelRaceGroup(
                        new SequentialCommandGroup(
                                new ShortDriveAway(),
                                new StopModules()
                        ),
                        new SequentialCommandGroup(
                                //this command will not end until the shooter is empty, this will pause the auto
                                new AutonShooterEmpty(),
                                new SetElevatorPosition(0)
                        )
                )
        );
    }
}
