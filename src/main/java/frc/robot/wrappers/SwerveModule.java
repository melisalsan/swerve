package frc.robot.wrappers;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    private static final double WHEEL_DIAMETER = 0.1016; // 4 inches (in meters)

    private static final double DRIVE_GEAR_RATIO = 6.75; // L2 modules

    // TODO: Get actual values, these values work but aren't ideal (especially for drive, they aren't accurate at all)
    private static final double AZIMUTH_KP = 0.01;
    private static final double AZIMUTH_KI = 0;
    private static final double AZIMUTH_KD = 0;
    private static final double DRIVE_KP = 0.11;
    private static final double DRIVE_KI = 0;
    private static final double DRIVE_KD = 0;

    private final PIDController azimuthController = new PIDController(AZIMUTH_KP, AZIMUTH_KI, AZIMUTH_KD);
    private final PIDController driveController = new PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD);

    private SwerveModuleState desiredState;

    private final NEO azimuthMotor;
    private final NEO driveMotor;

    private final CANCoder azimuthEncoder;

    /**
     * Instantiates a SwerveModule
     * @param azimuthMotor The NEO motor that controls the azimuth of the module.
     * @param driveMotor The NEO motor that controls the drive of the module.
     * @param azimuthEncoder The CANCoder that reads the azimuth of the module.
     */
    public SwerveModule(NEO azimuthMotor, NEO driveMotor, CANCoder azimuthEncoder) {
        this.azimuthMotor = azimuthMotor;
        this.driveMotor = driveMotor;
        this.azimuthEncoder = azimuthEncoder;

        // Configure the CANCoder
        this.azimuthEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    }

    /**
     * Sets the swerve modules target state. Needs to be called periodically
     * @param state Target state for the swerve module
     */
    public void setState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(getAzimuth()));

        desiredState = state;

        driveMotor.set(driveController.calculate(getSpeed(), state.speedMetersPerSecond / (WHEEL_DIAMETER * Math.PI)));

        double calc = azimuthController.calculate(getAzimuth(), state.angle.getDegrees());
        azimuthMotor.set(calc);
    }

    /**
     * Turns off the swerve module. Turns off motors
     */
    public void turnOffModule() {
        driveMotor.set(0);
        azimuthMotor.set(0);
    }

    /**
     * Gets the current drive speed of the module
     * @return The current drive speed of the module in meters per second
     */
    public double getSpeed() {
        return ((driveMotor.getVelocity() / 60) / DRIVE_GEAR_RATIO) * (WHEEL_DIAMETER * Math.PI);
    }

    /**
     * Gets the current azimuth of the module
     * @return The current azimuth of the module
     */
    public double getAzimuth() {
        return azimuthEncoder.getAbsolutePosition();
    }

    /**
     * Gets the current actual state of the module
     * @return The current actual state of the module
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getSpeed(), Rotation2d.fromDegrees(getAzimuth()));
    }

    /**
     * Gets the desired state of the module
     * @return The current desired state of the module
     */
    public SwerveModuleState getDesiredState() {
        return desiredState;
    }
}
