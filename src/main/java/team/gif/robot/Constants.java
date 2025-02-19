// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double DEBOUNCE_DEFAULT = 0.020;

    public static final class Drivetrain { // ToDo tune - remove when done
        //public static final double DRIVE_WHEEL_RADIUS = 0.05; // meters? Must be unit of velocity

        public static final double FRONT_LEFT_OFFSET = 79.435125;
        public static final double REAR_LEFT_OFFSET = -137.8125;
        public static final double FRONT_RIGHT_OFFSET = -20.09625;
        public static final double REAR_RIGHT_OFFSET = 155.126953125;


        //TODO: From mech team
        // Distance between centers of front and back wheels on robot
        public static final double TRACK_LENGTH = Units.inchesToMeters(24.899);

        // Distance between centers of left and right wheels on robot
        public static final double TRACK_WIDTH = Units.inchesToMeters(21.399);

        // location of wheels from center of robot using following axis
        //        +x
        //         ^
        //         |
        //  +y  <---
        public static final SwerveDriveKinematics DRIVE_KINEMATICS =
                new SwerveDriveKinematics(
                        new Translation2d(TRACK_LENGTH / 2, TRACK_WIDTH / 2), // front left
                        new Translation2d(TRACK_LENGTH / 2, -TRACK_WIDTH / 2), // front right
                        new Translation2d(-TRACK_LENGTH / 2, TRACK_WIDTH / 2), // back left
                        new Translation2d(-TRACK_LENGTH / 2, -TRACK_WIDTH / 2)); // back right

        public static final boolean kGyroReversed = false;

        // TODO: get feedback from driver
        public static final double COAST_DRIVE_PERC = 0.6;
        public static final double BOOST_DRIVE_PERC = 1;
        public static final double SLOW_DRIVE_PERC = 0.3;

        public static final double COAST_SPEED_METERS_PER_SECOND = COAST_DRIVE_PERC * ModuleConstants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND;
        public static final double BOOST_SPEED_METERS_PER_SECOND = BOOST_DRIVE_PERC * ModuleConstants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND;
        public static final double SLOW_SPEED_METERS_PER_SECOND = SLOW_DRIVE_PERC * ModuleConstants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND;

        public static double kMaxAccelerationMetersPerSecondSquared = 2;
    }

    public static final class ModuleConstants {
        public static final double COEFFICIENT_OF_FRICTION = 1;
        public static final double GEAR_RATIO = 6.75;
        public static final double ENCODER_CPR = 2048.0;
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
        public static final double DRIVE_ENCODER_ROT_2_METER = (Math.PI * WHEEL_DIAMETER_METERS) / GEAR_RATIO;
        public static final double DRIVE_ENCODER_RPM_2_METER_PER_SEC = DRIVE_ENCODER_ROT_2_METER / 60;

        public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = 4.5; //TODO: measure this

        public static final double PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 4 * Math.PI; //TODO

        public static final double TELE_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND; //TODO

        public static final double TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND = 3;

        public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND = 8 * Math.PI;

        public static final double TURNING_MOTOR_GEAR_RATIO = 1.0 / 18.0;

        public static final double TURNING_ENCODER_ROT_TO_RAD = TURNING_MOTOR_GEAR_RATIO * 2 * Math.PI;

        public static final double TURNING_ENCODER_RPM_2_RAD_PER_SECOND = TURNING_ENCODER_ROT_TO_RAD / 60;

        public static final class DrivetrainPID { //TODO: tuning pid
            public static final double frontLeftP = 0.55;// 0.35; //pBot 0.4 all P
            public static final double frontLeftFF = 0.01;//0.01; //pBot 0.01 all FF
            public static final double frontRightP = 0.55;
            public static final double frontRightFF = 0.01; //issa good
            public static final double rearLeftP = 0.55;//0.35;
            public static final double rearLeftFF = 0.01;//0.01;
            public static final double rearRightP = 0.55;//0.35; // 0.6
            public static final double rearRightFF = 0.01;//0.01;
            public static final SimpleMotorFeedforward fLDriveFF = new SimpleMotorFeedforward(0.16095, 2.3837, 0.077757);
            public static final SimpleMotorFeedforward fRDriveFF = new SimpleMotorFeedforward(0.1645, 2.3928, 0.074191);
            public static final SimpleMotorFeedforward rLDriveFF = new SimpleMotorFeedforward(0.10265, 2.3955, 0.22997);
            public static final SimpleMotorFeedforward rRDriveFF = new SimpleMotorFeedforward(0.13952 , 2.4217, 0.137);
        }
    }

    public static final class DrivetrainAuto {
        public static final double kP_FORWARD = 2;
        public static final double kP_ROTATION = 2.5;

        public static final PPHolonomicDriveController AUTO_DRIVE_CONTROLLER = new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(DrivetrainAuto.kP_FORWARD, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(Constants.DrivetrainAuto.kP_ROTATION, 0.0, 0.0)); // Rotation PID constants
    }

    // copied this from 2023
    public static final class DrivetrainMK3 {
        //public static final double DRIVE_WHEEL_RADIUS = 0.05; // meters? Must be unit of velocity

        public static final boolean kFrontLeftTurningEncoderReversed = false; //false
        public static final boolean kRearLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kRearRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveMotorReversed = false;
        public static final boolean kRearLeftDriveMotorReversed = false;
        public static final boolean kFrontRightDriveMotorReversed = false;
        public static final boolean kRearRightDriveMotorReversed = false;

        public static final boolean kFrontLeftTurningMotorReversed = false;
        public static final boolean kRearLeftTurningMotorReversed = true;
        public static final boolean kFrontRightTurningMotorReversed = true;
        public static final boolean kRearRightTurningMotorReversed = true;


        public static final double FRONT_LEFT_OFFSET = 42.8902;
        public static final double REAR_LEFT_OFFSET = 358.9453;
        public static final double FRONT_RIGHT_OFFSET =  255.4648;
        public static final double REAR_RIGHT_OFFSET = 199.0722;

        // Distance between centers of right and left wheels on robot
        public static final double TRACK_LENGTH = Units.inchesToMeters(22.5);

        // Distance between front and back wheels on robot
        public static final double TRACK_WIDTH = Units.inchesToMeters(23);

        // location of wheels from center of robot using following axis
        //        +x
        //         ^
        //         |
        //  +y  <---
        public static final SwerveDriveKinematics DRIVE_KINEMATICS =
                new SwerveDriveKinematics(
                        new Translation2d(TRACK_LENGTH / 2, TRACK_WIDTH / 2), // front left
                        new Translation2d(TRACK_LENGTH / 2, -TRACK_WIDTH / 2), // front right
                        new Translation2d(-TRACK_LENGTH / 2, TRACK_WIDTH / 2), // back left
                        new Translation2d(-TRACK_LENGTH / 2, -TRACK_WIDTH / 2)); // back right

        public static final boolean kGyroReversed = false;

        public static final double COAST_DRIVE_RPM = 2500; // 2750; //4800 demo speed //2750
        public static final double BOOST_DRIVE_RPM = 1675; // 1675 is max speed; was 1750;
        public static final double SLOW_DRIVE_RPM = 3500;

        public static final double COAST_SPEED_METERS_PER_SECOND = 0.60 * ModuleConstantsMK3.PHYSICAL_MAX_SPEED_METERS_PER_SECOND;

        public static final double BOOST_SPEED_METERS_PER_SECOND = BOOST_DRIVE_RPM *
                (Math.PI * ModuleConstantsMK3.WHEEL_DIAMETER_METERS) /
                (60.0 * ModuleConstantsMK3.GEAR_RATIO);

        public static final double SLOW_SPEED_METERS_PER_SECOND = SLOW_DRIVE_RPM *
                (Math.PI * ModuleConstantsMK3.WHEEL_DIAMETER_METERS) /
                (60.0 * ModuleConstantsMK3.GEAR_RATIO);
        public static double kMaxAccelerationMetersPerSecondSquared = 2;// TODO
    }

    public static final class ModuleConstantsMK3 {
        public static final double MAX_MODULE_ANGULAR_SPEED_RADIANS_PER_SECOND = 6 * (2 * Math.PI); //6
        public static final double MAX_MODULE_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 6 * (2 * Math.PI); //7
        public static final double GEAR_RATIO = 6.86; // TODO: need to ask someone
        public static final double ENCODER_CPR = 42; //This is for DRIVE
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3.78);
        public static final double WHEEL_CIRCUMFERENCE_METER = Units.inchesToMeters(12.25);
        public static final double DRIVE_ENCODER_ROT_2_METER = WHEEL_CIRCUMFERENCE_METER/ (GEAR_RATIO); //0.0004
        public static final double DRIVE_ENCODER_RPM_2_METER_PER_SEC = DRIVE_ENCODER_ROT_2_METER / 60;
        //4096.0 for talons
        public static final double kDriveEncoderDistancePerPulse =
                // Assumes the encoders are directly mounted on the wheel shafts
                (WHEEL_DIAMETER_METERS * Math.PI) / (double) ENCODER_CPR;

        public static final double kTurningEncoderDistancePerPulse =
                // Assumes the encoders are on a 1:1 reduction with the module shaft.
                (2 * Math.PI) / (double) ENCODER_CPR;

        public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = 4.5;

        public static final double PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 4 * Math.PI;
        public static final double TELE_DRIVE_MAX_SPEED_METERS_PER_SECOND = PHYSICAL_MAX_SPEED_METERS_PER_SECOND / 4;

        public static final double TELE_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND / 4;

        public static final double TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND = 10;

        public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND = 5 * Math.PI;

        public static final double TURNING_MOTOR_GEAR_RATIO = 12.8; //

        //            public static final double TURNING_ENCODER_ROT_TO_RAD = TURNING_MOTOR_GEAR_RATIO * 2 * Math.PI;
        public static final double TURNING_ENCODER_ROT_TO_RAD = 2 * Math.PI / 4096;

        public static final double TURNING_ENCODER_RPM_2_RAD_PER_SECOND = TURNING_ENCODER_ROT_TO_RAD / 60;

        public static final class DrivetrainPID {
            public static final double frontLeftP = 0.4; //pBot 0.4 all P
            public static final double frontLeftFF = 0.01; //pBot 0.01 all FF
            public static final double frontRightP = 0.4;
            public static final double frontRightFF = 0.01; //issa good
            public static final double rearLeftP = 0.4;
            public static final double rearLeftFF = 0.01;
            public static final double rearRightP = 0.4; // 0.6
            public static final double rearRightFF = 0.01;
            public static final SimpleMotorFeedforward fLDriveFF = new SimpleMotorFeedforward(0.16714, 2.7681, 0.41146);
            public static final SimpleMotorFeedforward fRDriveFF = new SimpleMotorFeedforward(0.10365, 2.7078, 0.49142);
            public static final SimpleMotorFeedforward rLDriveFF = new SimpleMotorFeedforward(0.10551, 2.8234, 0.48642);
            public static final SimpleMotorFeedforward rRDriveFF = new SimpleMotorFeedforward(0.073007, 2.75, 0.40028);
        }
    }

    public static final class Joystick {
        public static final double DEADBAND = 0.1;
    }
    public static final class MotorTemps {
        public static final double SHOOTER_MOTOR_TEMP = 70;
        public static final double INDEXER_MOTOR_TEMP = 70;
        public static final double DRIVETRAIN_MOTOR_TEMP = 85;
        public static final double COLLECTOR_MOTOR_TEMP = 70;
        public static final double ELEVATOR_MOTOR_TEMP = 70;
        public static final double CLIMBER_MOTOR_TEMP = 70;
    }
    public static final class LED {
        public static final int NUM_LEDS_TOTAL = 6;
    }

    public static final class Shooter {
        public static final double INDEX_PERCENT = 0.35; // todo: initial good - needs testing and final number
        public static final double SHOOT_PERCENT = 1.00; // todo: initial good - needs testing and final number
        public static final double STAGE_PERCENT = 0.50; // todo: initial good - needs testing and final number

        public static final double REEF_SENSOR_TARGET_DISTANCE_MM = 500;
        public static final double ALIGN_STRAFE_SPEED_MPS = 0.15; //Meters per Second
    }

    public static final class Elevator{
        public static final double PID_HOLD_FF = 0.025; //percent motor controller for simple FF pid
        public static final double ELEVATOR_KP = 0;
        public static final double ELEVATOR_KI = 0;
        public static final double ELEVATOR_KD = 0;
        public static final double ELEVATOR_KS = 0;
        public static final double ELEVATOR_KP_HOLD = 0;
        public static final double ELEVATOR_KI_HOLD = 0;
        public static final double ELEVATOR_KD_HOLD = 0;
        public static final double TICKS_PER_INCH = 1;
        public static final double ZERO_OFFSET_TICKS = 1;
        public static final double MAX_POS = 60;
        public static final double MIN_POS = 0;
        public static final int MAX_VELOCITY = 1;
        public static final double PID_TOLERANCE = 1;
        public static final double MAX_ACCELERATION = 1;
        public static final int REV_MAX_VELOCITY = 0;
        public static final double MIN_PERCENT_MANUAL = -.15;
        public static final double MAX_PERCENT_MANUAL = .15;
    }
}