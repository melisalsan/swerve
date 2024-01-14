package frc.robot.wrappers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class NEO {
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkMaxPIDController pidController;

    private int targetPosition = 0;
    private int targetVelocity = 0;

    /**
     * Creates a new NEO motor
     * @param id CANID of the SparkMax the NEO is connected to.
     */
    public NEO(int id) {
        this(id, false);
    }

    /**
     * Creates a new NEO motor
     * @param id CANID of the SparkMax the NEO is connected to.
     * @param mode The idle mode of the motor. If true, the motor will brake when not powered. If false, the motor will coast when not powered.
     */
    public NEO(int id, CANSparkMax.IdleMode mode) {
        this(id, false, mode);
    }

    /**
     * Creates a new NEO motor
     * @param id CANID of the SparkMax the NEO is connected to.
     * @param reversed Whether the motor is reversed or not.
     * @param mode The idle mode of the motor. If true, the motor will brake when not powered. If false, the motor will coast when not powered.
     */
    public NEO(int id, boolean reversed, CANSparkMax.IdleMode mode) {
        motor = new CANSparkMax(id, CANSparkMaxLowLevel.MotorType.kBrushless);

        motor.setInverted(reversed);
        setIdleMode(mode);

        encoder = motor.getEncoder();
        pidController = motor.getPIDController();
    }

    /**
     * Creates a new NEO motor
     * @param id CANID of the SparkMax the NEO is connected to.
     * @param reversed Whether the motor is reversed or not.
     */
    public NEO(int id, boolean reversed) {
        this(id, reversed, CANSparkMax.IdleMode.kBrake);
    }

    /**
     * Sets the NEO to brake or coast mode.
     * @param mode Idle mode, either brake mode or coast mode.
     */
    public void setIdleMode(CANSparkMax.IdleMode mode) {
        motor.setIdleMode(mode);
    }

    /**
     * Sets the NEO to run at a certain % of full power.
     * @param percent % of full power to run at.
     */
    public synchronized void set(double percent) {
        motor.set(percent);
    }

    /**
     * Sets the NEO to run at the voltage specified. Note that voltage compensation can affect this.
     * @param voltage Voltage to run motor at.
     */
    public synchronized void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    /**
     * Sets the target position for the NEO.
     * @param position Position to set the NEO to in rotations.
     * @param mode The control mode to use (Pure PIDFF or PIDFF + SmartMotion).
     */
    public synchronized void setPosition(double position, ControlMode mode) {
        setPosition(position, 0, mode);
    }

    /**
     * Sets the target position for the NEO.
     * @param position Position to set the NEO to in rotations.
     * @param arbitraryFeedForward Arbitrary feed forward to add to the motor output.
     * @param mode The control loop type to use (Pure PIDFF or PIDFF + SmartMotion).
     */
    public synchronized void setPosition(double position, double arbitraryFeedForward, ControlMode mode) {
        CANSparkMax.ControlType type;
        if (mode == ControlMode.PIDFF) {
            type = CANSparkMax.ControlType.kPosition;
        } else if (mode == ControlMode.SMART_MOTION) {
            type = CANSparkMax.ControlType.kSmartMotion;
        } else {
            throw new IllegalArgumentException("Invalid control mode");
        }

        pidController.setReference(position, type, 0, arbitraryFeedForward, SparkMaxPIDController.ArbFFUnits.kVoltage);
        targetPosition = (int) position;
    }

    /**
     * Sets the target velocity for the NEO.
     * @param velocity Velocity to set the NEO to in rotations per second.
     * @param mode The control loop type to use (Pure PIDFF or PIDFF + SmartMotion).
     */
    public synchronized void setVelocity(double velocity, ControlMode mode) {
        setVelocity(velocity, 0, mode);
    }

    /**
     * Sets the target velocity for the NEO.
     * @param velocity Velocity to set the NEO to in rotations per second.
     * @param arbitraryFeedForward Arbitrary feed forward to add to the motor output.
     * @param mode The control loop type to use (Pure PIDFF or PIDFF + SmartMotion).
     */
    public synchronized void setVelocity(double velocity, double arbitraryFeedForward, ControlMode mode) {
        if (velocity == 0) {
            set(0);
        } else {
            CANSparkMax.ControlType type;
            if (mode == ControlMode.PIDFF) {
                type = CANSparkMax.ControlType.kVelocity;
            } else if (mode == ControlMode.SMART_MOTION) {
                type = CANSparkMax.ControlType.kSmartVelocity;
            } else {
                throw new IllegalArgumentException("Invalid control mode");
            }

            pidController.setReference(velocity, type, 0, arbitraryFeedForward, SparkMaxPIDController.ArbFFUnits.kVoltage);
            targetVelocity = (int) velocity;
        }
    }

    /**
     * Sets the NEO to follow another NEO.
     * @param host The host NEO to follow. This NEO will mirror the host NEO's voltage outputs.
     */
    public void follow(NEO host) {
        motor.follow(host.getMotor(), getMotor().getInverted());
    }

    /**
     * Sets the NEO to follow another NEO.
     * @param host The host NEO to follow. This NEO will mirror the host NEO's voltage outputs.
     * @param inverted Sets the NEO to be inverted.
     */
    public void follow(NEO host, boolean inverted) {
        motor.follow(host.getMotor(), inverted);
    }

    /**
     * Gets the position of the NEO in rotations.
     * @return The position of the NEO in rotations relative to the last 0 position.
     */
    public double getPosition() {
        return encoder.getPosition();
    }

    /**
     * Gets the velocity of the NEO in rotations per second.
     * @return The instantaneous velocity of the NEO in rotations per second. NEO's use a filter in order to give more accurate data but this means that the output from this method will be 100ms behind reality.
     */
    public double getVelocity() {
        return encoder.getVelocity();
    }

    /**
     * Gets the target position of the NEO in rotations.
     * @return The target position of the NEO in rotations.
     */
    public int getTargetPosition() {
        return targetPosition;
    }

    /**
     * Gets the target velocity of the NEO in rotations per second.
     * @return The target velocity of the NEO in rotations per second.
     */
    public int getTargetVelocity() {
        return targetVelocity;
    }

    /**
     * Gets the current of the NEO in amps.
     * @return The current of the NEO in amps.
     */
    public double getCurrent() { return motor.getOutputCurrent(); }

    /**
     * Sets the current encoder position to whatever is specified in rotations.
     * @param position The position to set the encoder to in rotations.
     */
    public void setEncoder(double position) {
        encoder.setPosition(position);
    }

    /**
     * Sets the current limit.
     * @param limit The current limit the NEO will follow. The NEO will not pull more than this amount of current.
     */
    public void setCurrentLimit(int limit) {
        motor.setSmartCurrentLimit(limit);
    }

    /**
     * Burns all configured settings to the NEO's flash memory.
     */
    public void burn() {
        motor.burnFlash();
    }

    /**
     * Sets PID values.
     * @param kP The proportional gain constant for PIDFF control.
     * @param kI The integral gain constant for PIDFF control.
     * @param kD The derivative gain constant for PIDFF control.
     * @param kIZ The I-Zone constant for PIDFF control. The I term will not start accumulating until the error is less than this value.
     * @param kFF The feedforward gain constant for PIDFF control.
     */
    public void configurePIDFF(double kP, double kI, double kD, double kIZ, double kFF) {
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setIZone(kIZ);
        pidController.setFF(kFF);
    }

    /**
     * Configures SmartMotion.
     * @param maxVelocity The maximum velocity the NEO will run at in rotations per second when being controlled by SmartMotion.
     * @param maxAcceleration The maximum acceleration the NEO will be subjected to in rotations per second per second when being controlled by SmartMotion.
     */
    public void configureSmartMotion(double maxVelocity, double maxAcceleration) {
        configureSmartMotion(maxVelocity, maxAcceleration, SparkMaxPIDController.AccelStrategy.kTrapezoidal);
    }

    /**
     *
     * @param maxVelocity The maximum velocity the NEO will run at in rotations per second when being controlled by SmartMotion.
     * @param maxAcceleration The maximum acceleration the NEO will be subjected to in rotations per second per second when being controlled by SmartMotion.
     * @param accelStrategy The acceleration strategy the NEO will use. It's velocity over time will either follow a trapezoidal shape or an s-curve.
     */
    public void configureSmartMotion(double maxVelocity, double maxAcceleration, SparkMaxPIDController.AccelStrategy accelStrategy) {
        pidController.setSmartMotionMaxVelocity(maxVelocity, 0);
        pidController.setSmartMotionMaxAccel(maxAcceleration, 0);
        pidController.setSmartMotionAccelStrategy(accelStrategy, 0);
    }

    /**
     * Sets up voltage compensation.
     * @param voltage Voltage to compensate for. Will set the motor voltage relative to this value instead of the battery voltage.
     */
    public void configureVoltageComp(double voltage) {
        motor.enableVoltageCompensation(voltage);
    }

    /**
     * Gets the current applied % output from the motor.
     * @return The current applied % output from the motor.
     */
    public double getOutput() {
        return motor.getAppliedOutput();
    }

    /**
     * Clamps the applied power output for the NEO to this range.
     * @param min Minimum power to output
     * @param max Maximum power to output.
     */
    public void setOutputRange(double min, double max) {
        pidController.setOutputRange(min, max);
    }

    /**
     * Gets the proportional gain constant for PIDFF controller.
     * @return The proportional gain constant for PIDFF controller.
     */
    public double getP() {
        return pidController.getP();
    }

    /**
     * Gets the integral gain constant for PIDFF controller.
     * @return The integral gain constant for PIDFF controller.
     */
    public double getI() {
        return pidController.getI();
    }

    /**
     * Gets the derivative gain constant for PIDFF controller.
     * @return The derivative gain constant for PIDFF controller.
     */
    public double getD() {
        return pidController.getD();
    }


    /**
     * Gets the I-Zone constant for PIDFF controller.
     * @return The I-Zone constant for PIDFF control.
     */
    public double getIZ() {
        return pidController.getIZone();
    }

    /**
     * Gets the feedforward gain constant for PIDFF controller.
     * @return The feedforward gain constant for PIDFF controller.
     */
    public double getFF() {
        return pidController.getFF();
    }

    /**
     * Gets the CANID of the SparkMax running this motor.
     * @return The CANID of the SparkMax running this motor.
     */
    public int getID() {
        return motor.getDeviceId();
    }

    /**
     * Gets the CANSparkMax object behind this motor.
     * @return The CANSparkMax object behind this motor.
     */
    public CANSparkMax getMotor() {
        return motor;
    }

    /**
     * The two control modes for the NEO (pure PIDFF and PIDFF + SmartMotion).
     */
    public enum ControlMode {
        PIDFF,
        SMART_MOTION
    }
}
