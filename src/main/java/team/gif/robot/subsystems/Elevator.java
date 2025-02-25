package team.gif.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
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
    private boolean softLimitEnabled;

    private double stallLastPosition = 0;
    private int stallCount = 0;

    public Elevator() {
        elevatorMotor = new TalonFX(RobotMap.ELEVATOR_MOTOR_ID);
        configElevatorTalon();
        zeroEncoder();

        stallLastPosition = getPosition();
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
        move(Constants.Elevator.PID_HOLD_FF);
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
        final MotionMagicVoltage elevatorMotionMagic = new MotionMagicVoltage(position);
        elevatorMotor.setControl(elevatorMotionMagic);
    }

    /**
     * Sets the cruise velocity of the elevator in ticks per 100ms
     *
     * @param rotationsPersecond the cruise velocity of the elevator in Rotations Per second
     */
    public void setCruiseVelocity(int rotationsPersecond) {
        // new unit (rot/sec)
        final VelocityDutyCycle elevatorVelocity = new VelocityDutyCycle(0);
        elevatorMotor.setControl(elevatorVelocity);
    }

    public void setElevatorTargetPos(double pos) {
        elevatorTargetPos = pos;
    }

    public boolean isMotionMagicFinished() {
        return Math.abs(PIDError()) < Constants.Elevator.PID_TOLERANCE;
    }

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
            TalonFXConfiguration config = new TalonFXConfiguration();

            config.SoftwareLimitSwitch.ReverseSoftLimitEnable = state;
            elevatorMotor.getConfigurator().apply(config);
            softLimitEnabled = state;
        }
    }

    public double getValue() {
        return elevatorMotor.getMotorVoltage().getValueAsDouble();
    }

    /**
     * Zeroes the elevator encoder
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
        if (Math.abs(getPosition()) - stallLastPosition < 0.4 && getOutputPercent() > Constants.Elevator.PID_HOLD_FF * 1.25) {
            //don't reset stall last position here
            //because it could just be moving slowly
            stallCount++;
//            System.out.println("stalling");
        } else {
            stallLastPosition = getPosition();
            stallCount = 0;
        }

//        if( stallCount >= 10)
//            System.out.println("       stalled");
        return stallCount >= 10;
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
        softLimitEnabled = true;

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