package team.gif.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;

import static com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class Elevator extends SubsystemBase {
    public final TalonFX elevatorMotor;

    private boolean elevatorManualMode = false;
    private double elevatorTargetPos;

    // changing config is very time expensive. Store last config values to compare
    private boolean softLimitEnabled;
    private double currVelocity;
    private double currAcceleration;

    // used for determining if elevator motor is stalled
    private double stallLastPosition;
    private int stallCount;
    private boolean elevatorEnabled;

    public Elevator() {
        elevatorMotor = new TalonFX(RobotMap.ELEVATOR_MOTOR_ID);
        configElevatorTalon();
        zeroEncoder();

        softLimitEnabled = true;
        currVelocity = Constants.Elevator.MAX_VELOCITY;
        currAcceleration = Constants.Elevator.MAX_ACCELERATION;

        stallLastPosition = getPosition();
        stallCount = 0;

        elevatorEnabled = true;
    }

    /**
     * Sets the elevator to either manual or motion magic (and PID) mode
     *
     * @param manual set to true if manual mode, false if motion magic mode
     */
    public void setElevatorManualMode(boolean manual) {
        this.elevatorManualMode = manual;
    }

    /**
     * Moves the elevator with a percent output
     *
     * @param percent percent of max speed to run the elevator motor
     */
    public void move(double percent) {
        elevatorMotor.set(percent);
    }

    /**
     * Holds the elevator at its given position with PID
     */
    public void PIDHold() {
        // Only command the motor whwen it is needs to be used (i.e. not near 0). This reduces the number
        // of messages to the motor, reducing CAN utilization.
        // This is not a true PID command, but rather a simple Feed Forward to hold the elevator up. Trur PID
        // is overkill for this application
        if (getPosition() >= 0.8) {
            move(Constants.Elevator.PID_HOLD_FF);
        }

        //PositionDutyCycle elevatorPos = new PositionDutyCycle(0);
        //elevatorMotor.setPosition(1, 0);
        // the elevator needs a different kF when it is lower to the ground, otherwise it doesn't stay at the position
        //elevatorMotor.config_kF(1, Constants.Elevator.F_HOLD);
        //elevatorMotor.setControl(elevatorPos.withPosition(elevatorTargetPos)); // closed loop position control
    }

    /**
     * Get the position of the elevator in encoder ticks
     *
     * @return the position of the elevator in encoder ticks
     */
    public double getPosition() {
        return elevatorMotor.getPosition().getValueAsDouble();
    }

    /**
     * Get the position of the elevator in inches
     *
     * @return the position of the elevator in inches
     */
    public double getPositionInches() {
        return (getPosition() + Constants.Elevator.ZERO_OFFSET_TICKS) / Constants.Elevator.TICKS_PER_INCH;
    }

    /**
     * Convert a position in inches to encoder ticks
     *
     * @param inches the position in inches
     * @return the position in encoder ticks
     */
    public double inchesToPos(int inches) {
        return inches * Constants.Elevator.TICKS_PER_INCH - Constants.Elevator.ZERO_OFFSET_TICKS;
    }

    /**
     * @param inches imperial unit of measurement
     * @return Inches in units of ticks
     */
    public double inchesToTicks(int inches) {
        return inches * Constants.Elevator.TICKS_PER_INCH;
    }


    /**
     * Get the target elevator position
     *
     * @return the target elevator position
     */
    public double getTargetPosition() {
        return elevatorTargetPos;
    }

    /**
     * Get the PID error on the elevator position
     *
     * @return the PID error on the elevator position
     */
    public double PIDError() {
        return Math.abs(getPosition() - elevatorTargetPos);
    }

    /**
     * Sets the elevator to a given position with Motion Magic
     *
     * @param position the position to set the elevator to
     */
    public void setMotionMagic(double position) {

        // Need to set this for PIDError result
        setElevatorTargetPos(position);

        final MotionMagicVoltage elevatorMotionMagic = new MotionMagicVoltage(position);
        elevatorMotor.setControl(elevatorMotionMagic);
    }

    /**
     * Update the motor configuration for Motion Magic
     * All parameters must be set, no default
     * Typically used when we want to use different configuratinos for going up vs going down
     *
     * @param velocity max velocity to use for motion magic config
     * @param acceleration max acceleration to use for motion magic config
     */
    public void updateMotionMagicParms(double velocity, double acceleration) {
        // changing the config of the talon is very expensive, causes loop overruns
        // only update the talon config if it has changed since the last setting
        if (currVelocity != velocity || currAcceleration != acceleration) {
            MotionMagicConfigs config = new MotionMagicConfigs();
            config.MotionMagicCruiseVelocity = velocity;
            config.MotionMagicAcceleration = acceleration;
            elevatorMotor.getConfigurator().apply(config);

            // update the current values for future comparison
            currVelocity = velocity;
            currAcceleration = acceleration;
        }
    }

    /**
     * Confgure the elevator motor for motion magic going up
     */
    public void configMotionMagicUp() {
        updateMotionMagicParms(Constants.Elevator.MAX_VELOCITY, Constants.Elevator.MAX_ACCELERATION);
    }

    /**
     * Confgure the elevator motor for motion magic going down
     */
    public void configMotionMagicDown() {
        updateMotionMagicParms(Constants.Elevator.REV_MAX_VELOCITY, Constants.Elevator.REV_MAX_ACCELERATION);
    }

    /**
     * Disable the elevator (by setting all the motion magic pid values to 0)
     */
    public void disableElevator() {
        // Slot 0 (Motion Magic) PID values
        Slot0Configs config = new Slot0Configs();

        config.kP = 0;
        config.kI = 0;
        config.kD = 0;
        config.kS = 0;
        elevatorMotor.getConfigurator().apply(config);

        elevatorEnabled = false;
    }

    /**
     * Enables the elevator after the disableElevator function was called
     */
    public void enableElevator() {
        // Slot 0 (Motion Magic) PID values
        Slot0Configs config = new Slot0Configs();

        config.kP = Constants.Elevator.ELEVATOR_KP;
        config.kI = Constants.Elevator.ELEVATOR_KI;
        config.kD = Constants.Elevator.ELEVATOR_KD;
        config.kS = Constants.Elevator.ELEVATOR_KS;
        elevatorMotor.getConfigurator().apply(config);

        elevatorEnabled = true;
    }

    /**
     *
     * @return returns true if elevator is unlocked
     */
    public boolean getElevatorUnlocked(){
        return elevatorEnabled;
    }


    /**
     * sets the class local elevator target position
     * Used for comparing current position to target position to
     * determine if the elevator has reached its desired position
     *
     * @param pos target position of the elevator
     */
    public void setElevatorTargetPos(double pos) {
        elevatorTargetPos = pos;
    }

    /**
     * Determines if Motion Magic has reached its desired position within tolerance
     *
     * @return true Motion Magic has reached its desired position within tolerance, false if not
     */
    public boolean isMotionMagicFinished() {
        return Math.abs(PIDError()) < Constants.Elevator.MOTION_MAGIC_TOLERANCE;
    }

    /**
     * Determines if the elevator is up and at its target position. Will return false if
     * the elevator is all the way down at the collector position
     *
     * @return true if the elevator is up ready to shoot, false if not
     */
    public boolean isReadyToShoot() {
        if (getTargetPosition() == Constants.Elevator.COLLECTOR_POSITION) {
            return false;
        }

        return Math.abs(getTargetPosition() - getPosition()) < Constants.Elevator.SHOOT_TOLERANCE;
    }

    /**
     * gets the motor output voltage
     *
     * @return voltage (in units of volts)
     */
    public double getOutputVoltage() {
        return elevatorMotor.getMotorVoltage().getValueAsDouble();
    }

    /**
     * Gets the percent output of the elevator motor
     *
     * @return the percent output of the elevator motor
     */
    public double getOutputPercent() {
        return elevatorMotor.get();
    }

    /**
     * Gets the velocity of the elevator in ticks per 100ms
     *
     * @return the velocity of the elevator in ticks per 100ms
     */
    public double getVelTPS() {
        return elevatorMotor.getVelocity().getValueAsDouble();
    }

    /**
     * Gets the current of the elevator motor
     */
    public double getCurrent() {
        return elevatorMotor.getSupplyCurrent().getValueAsDouble();
    }

    /**
     * Enables/disables the reverse soft limit of the elevator
     */
    public void enableLowerSoftLimit(boolean state) {
        // changing the config of the talon is very expensive, causes loop overruns
        // only update the talon config if it has changed since the last setting
        if (softLimitEnabled != state) {
            SoftwareLimitSwitchConfigs config = new SoftwareLimitSwitchConfigs();

            config.ReverseSoftLimitEnable = state;
            elevatorMotor.getConfigurator().apply(config);
            softLimitEnabled = state;
        }
    }

    /**
     * Zeros the elevator encoder
     */
    public void zeroEncoder() {
        elevatorMotor.setPosition(0);
    }

    /**
     * Indicates elevator motor temp as true or false
     *
     * @return True if temp equal or greater than set temp limit, otherwise return false.
     */
    public boolean isElevatorMotorHot(){
        return elevatorMotor.getDeviceTemp().getValueAsDouble() >= Constants.MotorTemps.ELEVATOR_WARNING_MOTOR_TEMP;
    }

    public boolean isStalled() {
        //If the elevator has moved less than 1 rotation
        //since the last cycle, increment the stall count
        //Don't detect a stall if outputting less than feedforward
        //This prevents detection during PIDHold
//        System.out.println(Math.abs(getPosition()) - stallLastPosition);
//        if (Math.abs(getPosition()) - stallLastPosition < 0.4 && getOutputPercent() > Constants.Elevator.PID_HOLD_FF * 1.25) {
//            //don't reset stall last position here
//            //because it could just be moving slowly
//            stallCount++;
//        } else {
//            stallLastPosition = getPosition();
//            stallCount = 0;
//        }

//        if( stallCount >= 10)
//          return stallCount >= 10;
        return false;
    }

    /**
     * Configures the elevator Talon
     */
    private void configElevatorTalon() {
        // set to brake mode
        elevatorMotor.setNeutralMode(NeutralModeValue.Brake);

        TalonFXConfiguration config = new TalonFXConfiguration();

        // Slot 1 (PID_HOLD) configuration
        config.Slot1.kP = Constants.Elevator.ELEVATOR_KP_HOLD;
        config.Slot1.kI = Constants.Elevator.ELEVATOR_KI_HOLD;
        config.Slot1.kD = Constants.Elevator.ELEVATOR_KD_HOLD;

        // invert direction
        config.MotorOutput.Inverted = Clockwise_Positive;

        // Soft Limits
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Elevator.MIN_POS;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Elevator.MAX_POS;


        /*   Motion Magic configuration  */
        // Motion magic config values
        config.MotionMagic.MotionMagicAcceleration = Constants.Elevator.MAX_ACCELERATION;
        config.MotionMagic.MotionMagicCruiseVelocity = Constants.Elevator.MAX_VELOCITY;

        // Slot 0 (Motion Magic) PID values
        config.Slot0.kP = Constants.Elevator.ELEVATOR_KP;
        config.Slot0.kI = Constants.Elevator.ELEVATOR_KI;
        config.Slot0.kD = Constants.Elevator.ELEVATOR_KD;
        config.Slot0.kS = Constants.Elevator.ELEVATOR_KS;

        // Write these configs to the TalonFX
        elevatorMotor.getConfigurator().apply(config);

        /* from 2023 - do we need?
        MotorOutputConfigs elevatorOutputConfigs = new MotorOutputConfigs();
        elevatorOutputConfigs.withPeakReverseDutyCycle(0);
        elevatorOutputConfigs.withPeakReverseDutyCycle(0);
        elevatorMotor.getConfigurator().apply(elevatorOutputConfigs);
        */
    }

    private SysIdRoutine getSysIdRoutine(){
        MutVoltage voltMut = Volts.mutable(0);
        MutDistance posMut = Meters.mutable(0);
        MutLinearVelocity vMut= MetersPerSecond.mutable(0);

        return new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                    voltage -> {
                        elevatorMotor.setVoltage(voltage.baseUnitMagnitude());
                    },
                    sysIdRoutineLog -> {
                        sysIdRoutineLog.motor("Elevator")
                                .voltage(voltMut.mut_replace(elevatorMotor.getMotorVoltage().getValueAsDouble(), Volts))
                                .linearPosition(posMut.mut_replace(elevatorMotor.getPosition().getValueAsDouble(), Meters))
                                .linearVelocity(vMut.mut_replace(elevatorMotor.getVelocity().getValueAsDouble(),MetersPerSecond));
                    },
                        this));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction){
        return getSysIdRoutine().quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction){
        return getSysIdRoutine().dynamic(direction);
    }

}