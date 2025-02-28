// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import team.gif.robot.commands.elevator.SetElevatorPosition;
import team.gif.robot.commands.shooter.AutonShootReset;
import team.gif.robot.commands.shooter.Shoot;
import team.gif.robot.commands.shooter.AutonAutoTarget;
import team.gif.robot.commands.shooter.StageCoral;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;
import java.util.stream.Stream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    private final SendableChooser<Command> autoChooser;


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        // register commands used in PathPlanner
        NamedCommands.registerCommand("Shoot", new Shoot());
        NamedCommands.registerCommand("AutoElevatorLvl4", new SetElevatorPosition(Constants.Elevator.LEVEL_4_POSITION));
        NamedCommands.registerCommand("AutonAutoShoot", new AutonAutoTarget());
        NamedCommands.registerCommand("AutonShootReset", new AutonShootReset());
        NamedCommands.registerCommand("StageCoral", new StageCoral());
        NamedCommands.registerCommand("Print Me", new InstantCommand(() -> System.out.println("Printing here")));
        NamedCommands.registerCommand("StopDrive", new InstantCommand(Robot.swerveDrive::stopDrive));


        // Configure the trigger bindings
        configureBindings();

        // builds auto chooser
        // AutoBuilder.buildAutoChooser pulls file list (*.auto) from {project}/src/main/deploy/pathplanner/autos/deploy,
        // creates a chooser list, and builds the paths, all in this single command
        // Takes the default auto path as a parameter
//        autoChooser = AutoBuilder.buildAutoChooser("");
//        autoChooser = build2338AutoChooser("",(stream) -> stream);
        autoChooser = build2338AutoChooser((stream) -> stream);
        SmartDashboard.putData("Auto", autoChooser);
    }

    // Copied code from WPILIB AutoBuilder.java and added sort feature
    // Also moves None to the top and renames it "** None **"
    public static SendableChooser<Command> build2338AutoChooser(
        Function<Stream<PathPlannerAuto>, Stream<PathPlannerAuto>> optionsModifier) {

        if (!AutoBuilder.isConfigured()) {
            throw new RuntimeException(
                "AutoBuilder was not configured before attempting to build an auto chooser");
        }

        SendableChooser<Command> chooser = new SendableChooser<>();

        // get list from .../deploy directory
//        autoNames = AutoBuilder.getAllAutoNames();
//        autoNames.sort(null); // this is the line added to sort the list (retrieved from the .../deploy directory)

        // create list manually to control sort order
        List<String> autoNames = new ArrayList<>();
        autoNames.add("Mobility");
        autoNames.add("L-J4-L4");
        autoNames.add("C-H4");
        autoNames.add("LC-J4-L4");

//        PathPlannerAuto defaultOption = null;
        List<PathPlannerAuto> options = new ArrayList<>();

        for (String autoName : autoNames) {
            PathPlannerAuto auto = new PathPlannerAuto(autoName);

//            if (!defaultAutoName.isEmpty() && defaultAutoName.equals(autoName)) {
//                defaultOption = auto;
//            } else {
                options.add(auto);
//            }
        }

//        if (defaultOption == null) {
            chooser.setDefaultOption("** None **", Commands.none());
//        } else {
//            chooser.setDefaultOption(defaultOption.getName(), defaultOption);
//            chooser.addOption("** None **", Commands.none());
//        }

        optionsModifier
            .apply(options.stream())
            .forEach(auto -> chooser.addOption(auto.getName(), auto));

        return chooser;
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
    }
}