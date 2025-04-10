package team.gif.robot;

public abstract class RobotMap {
    // Controllers
    public static final int DRIVER_CONTROLLER_ID = 0;
    public static final int AUX_CONTROLLER_ID = 1;
    public static final int TEST_CONTROLLER_ID = 2;

    // SwerveDrivetrain IDs
    /*                As viewed from center of bot
     *              Drive Motor        Turn Motor           CAN
     *            Left(1) Right(2)  Left(3) Right(4)  Left(5) Right(6)
     *   Front(1)   11     12          13      14       15      16
     *   Rear(2)    21     22          23      24       25      26
     *
     */
    public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 11;
    public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 12;
    public static final int REAR_LEFT_DRIVE_MOTOR_ID = 21;
    public static final int REAR_RIGHT_DRIVE_MOTOR_ID = 22;

    public static final int FRONT_LEFT_TURNING_MOTOR_ID = 13;
    public static final int FRONT_RIGHT_TURNING_MOTOR_ID = 14;
    public static final int REAR_LEFT_TURNING_MOTOR_ID = 23;
    public static final int REAR_RIGHT_TURNING_MOTOR_ID = 24;

    public static final int FRONT_LEFT_CANCODER_ID = 15;
    public static final int FRONT_RIGHT_CANCODER_ID = 16;
    public static final int REAR_LEFT_CANCODER_ID = 25;
    public static final int REAR_RIGHT_CANCODER_ID = 26;

    public static final int PIGEON_ID = 9;

    // MK3 RobotMap
    public static final int PRACTICE_REAR_LEFT_DRIVE_ID = 34;//1;
    public static final int PRACTICE_REAR_LEFT_TURN_ID = 9;//21;
    public static final int PRACTICE_REAR_RIGHT_DRIVE_ID = 1; //20;
    public static final int PRACTICE_REAR_RIGHT_TURN_ID = 32;//9;
    public static final int PRACTICE_FRONT_LEFT_DRIVE_ID = 14;//45;
    public static final int PRACTICE_FRONT_LEFT_TURN_ID = 7;//7;
    public static final int PRACTICE_FRONT_RIGHT_DRIVE_ID = 20;//14;
    public static final int PRACTICE_FRONT_RIGHT_TURN_ID = 31;//8;

    //Shooter
    public static final int SHOOTER_MOTOR_ID = 41;
    public static final int INDEXER_MOTOR_ID = 42;
    public static final int SERVO_PORT_ID = 0;

    public static final int INDEXER_GP_SENSOR_PORT = 0;
    public static final int EXIT_GP_SENSOR_PORT = 1;
    public static final int REEF_LEFT_SENSOR_ID = 51;
    public static final int REEF_RIGHT_SENSOR_ID = 52;
    public static final int COLLECTOR_SIDE_SENSOR_ID = 53;

    //Elevator
    public static final int ELEVATOR_MOTOR_ID = 44;

    //Climber
    public static final int CLIMBER_MOTOR_ID = 43;
    public static final int CLIMBER_SOLENOID_IN_PORT = 5;
    public static final int CLIMBER_SOLENOID_OUT_PORT = 4;

    //Grabber
    public static final int GRABBER_SOLENOID_PORT = 7;

    //UI
    public static final class UI {
        public static final String SHOOTER_PERC = "Shooter %";
        public static final String SHOOTER_LEVEL_3_PERC = "Level 3 %";
        public static final String ELEVATOR_LEVEL_3 = "Level 3 Pos";
        public static final String SHOOTER_LEVEL_1_PERC = "Level 1 %";
        public static final String STAGE_PERC = "Stage %";
        public static final String INDEXER_PERC = "Indexer Speed";
    }

    //Compressers
    public static final int COMPRESSER = 1;
}
