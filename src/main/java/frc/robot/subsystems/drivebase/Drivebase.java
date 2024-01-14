package frc.robot.subsystems.drivebase;

import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Profile;
import frc.robot.Utils;
import frc.robot.constants.CANID;
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.wrappers.Limelight;
import frc.robot.wrappers.NEO;
import frc.robot.wrappers.SwerveModule;

public class Drivebase extends Subsystem {
    private static final Shooter shooter = Shooter.getInstance();
    private static final Limelight limelight = shooter.getLimelight();
    private static final Profile profile = Profile.getProfile();

    private static Drivebase instance;

    public static Drivebase getInstance() {
        if (instance == null) {
            instance = new Drivebase();
        }
        return instance;
    }

    private Drivebase() {}

    private final AHRS navX = new AHRS();

    // frontLeft
    private final NEO frontLeftAzimuth = new NEO(CANID.FRONT_LEFT_AZIMUTH);
    private final NEO frontLeftDrive = new NEO(CANID.FRONT_LEFT_DRIVE);
    private final CANCoder frontLeftAzimuthEncoder = new CANCoder(CANID.FRONT_LEFT_AZIMUTH_ENCODER);
    private final SwerveModule frontLeftModule = new SwerveModule(frontLeftAzimuth, frontLeftDrive, frontLeftAzimuthEncoder);

    // frontRight
    private final NEO frontRightAzimuth = new NEO(CANID.FRONT_RIGHT_AZIMUTH);
    private final NEO frontRightDrive = new NEO(CANID.FRONT_RIGHT_DRIVE);
    private final CANCoder frontRightAzimuthEncoder = new CANCoder(CANID.FRONT_RIGHT_AZIMUTH_ENCODER);
    private final SwerveModule frontRightModule = new SwerveModule(frontRightAzimuth, frontRightDrive, frontRightAzimuthEncoder);

    // backLeft
    private final NEO backLeftAzimuth = new NEO(CANID.BACK_LEFT_AZIMUTH);
    private final NEO backLeftDrive = new NEO(CANID.BACK_LEFT_DRIVE, true);
    private final CANCoder backLeftAzimuthEncoder = new CANCoder(CANID.BACK_LEFT_AZIMUTH_ENCODER);
    private final SwerveModule backLeftModule = new SwerveModule(backLeftAzimuth, backLeftDrive, backLeftAzimuthEncoder);

    // backRight
    private final NEO backRightAzimuth = new NEO(CANID.BACK_RIGHT_AZIMUTH);
    private final NEO backRightDrive = new NEO(CANID.BACK_RIGHT_DRIVE, true);
    private final CANCoder backRightAzimuthEncoder = new CANCoder(CANID.BACK_RIGHT_AZIMUTH_ENCODER);
    private final SwerveModule backRightModule = new SwerveModule(backRightAzimuth, backRightDrive, backRightAzimuthEncoder);

    // frontLeft, frontRight, backLeft, backRight
    private final SwerveModule[] swerveModules = new SwerveModule[] {
            frontLeftModule,
            frontRightModule,
            backLeftModule,
            backRightModule
    };

    // Wheel base length in meters (from center of module to center of other module)
    private final double wheelBaseLen = 0.62865;

    private final Translation2d frontLeftLocation = new Translation2d(-wheelBaseLen / 2, wheelBaseLen / 2);
    private final Translation2d frontRightLocation = new Translation2d(-wheelBaseLen / 2, -wheelBaseLen / 2);
    private final Translation2d backLeftLocation = new Translation2d(wheelBaseLen / 2, wheelBaseLen / 2);
    private final Translation2d backRightLocation = new Translation2d(wheelBaseLen / 2, -wheelBaseLen / 2);

    private final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    private final Pose2d startingPosition = new Pose2d(0, 0, new Rotation2d());

    // TODO: Impl and test odometry, I didn't have time to work on this before Battle at the Border
    private final SwerveDriveOdometry swerveDriveOdometry = new SwerveDriveOdometry(swerveDriveKinematics, getGyroAngle(), startingPosition);

    // TODO: Get actual values, these temp values work but aren't ideal
    /**
     * The max speed of the robot in meters per second
     */
    public static final double MAX_VELOCITY = 4;
    /**
     * The max angular velocity of the robot in rotations per second
     */
    public static final double MAX_ROTATIONS_PER_SECOND = 1;
    /**
     * The max acceleration of the robot in meters per second squared
     */
    public static final double MAX_ACCELERATION = 6;

    // TODO: Impl a system to set this depending on the auto
    /**
     * The angle the robot is facing at the start of the match in degrees
     */
    public static final double STARTING_ANGLE_OFFSET = 0;

    private double ticksSinceLastInput = 0;

    private final Field2d field = new Field2d();

    @Override
    public void robotInit() {
        // Reset the navX and set the forward direction for the robot
        navX.reset();
        navX.setAngleAdjustment(STARTING_ANGLE_OFFSET);

        // Puts the field for odometry on the smart dashboard, currently untested
        SmartDashboard.putData(field);
    }

    @Override
    public void teleopPeriodic() {
        // Get the joystick inputs
        double translateX = Utils.deadband(profile.TRANSLATE_X.getAxisState());
        double translateY = Utils.deadband(-profile.TRANSLATE_Y.getAxisState());
        double rotate = Utils.deadband(profile.ROTATE.getAxisState());

        // If the driver isn't driving, lock the swerve modules into the locked position after some time, otherwise, drive
        if (translateX == 0 && translateY == 0 && rotate == 0) {
            // Count ticks the driver hasn't been driving
            ticksSinceLastInput++;

            // If the driver hasn't been driving for long enough, lock the swerve modules into the locked position
            if (ticksSinceLastInput >= 30) {
                lockModules();
            } else {
                // Keep the modules' azimuth but stop driving
                frontLeftModule.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(frontLeftModule.getAzimuth())));
                backRightModule.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(backRightModule.getAzimuth())));
                frontRightModule.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(frontRightModule.getAzimuth())));
                backLeftModule.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(backLeftModule.getAzimuth())));
            }
        } else {
            ticksSinceLastInput = 0;

            // Drive the robot
            drive(translateX, translateY, rotate);
        }
    }

    @Override
    public void robotPeriodic() {
        // Update odometry
        swerveDriveOdometry.update(getGyroAngle(), frontLeftModule.getState(), frontRightModule.getState(), backLeftModule.getState(), backRightModule.getState());

        // Currently disabled but if the robot is moving slow enough, use the limelight to reset the odometry
        /*if (getOverallSpeed() < 0.1) {
            Pose2d llPose = calculatePoseFromLimelight();
            swerveDriveOdometry.resetPosition(llPose, getGyroAngle());
            // Use below code if you only want to reset odo with ll when innacuracy is detected
            /*
            Pose2d llPose = calculatePoseFromLimelight();
            Pose2d odoPose = swerveDriveOdometry.getPoseMeters();

            double xOffset = Math.abs(llPose.getX() - odoPose.getX());
            double yOffset = Math.abs(llPose.getY() - odoPose.getY());
            double rotOffset = Math.abs(llPose.getAzimuth().getDegrees() - odoPose.getAzimuth().getDegrees());

            if (xOffset > ODO_MAX_TRANS_ERROR || yOffset > ODO_MAX_TRANS_ERROR || rotOffset > ODO_MAX_ROT_ERROR) {
                // Might also be Rotation2d.fromDegrees(0) but idk
                swerveDriveOdometry.resetPosition(llPose, getGyroAngle());
            }*/
        //}
    }

    @Override
    public void disabledInit() {
        // When the robot's off, turn off all module motors
        stopModules();
    }

    /**
     * Drives the robot (field oriented control)
     * @param translateX Field relative X speed (% top speed)
     * @param translateY Field relative Y speed (% top speed)
     * @param rotate Rotation speed (% top speed)
     */
    public void drive(double translateX, double translateY, double rotate) {
        // Get the desired robot relative speeds from the desired field relative speeds
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(translateX * Drivebase.MAX_VELOCITY, translateY * Drivebase.MAX_VELOCITY, (rotate * Drivebase.MAX_ROTATIONS_PER_SECOND) * (Math.PI * 2), getGyroAngle());

        // Convert the robot relative speeds to swerve module states
        SwerveModuleState[] moduleStates = getSwerveDriveKinematics().toSwerveModuleStates(chassisSpeeds);

        // Normalize the speeds so that no module is going faster than the max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Drivebase.MAX_VELOCITY);

        // Set every swerve modules state to the desired state
        for (int module = 0; module < moduleStates.length; module++) {
            SwerveModule swerveModule = swerveModules[module];

            swerveModule.setState(moduleStates[module]);
        }
    }

    /**
     * Drives the robot (robot oriented control)
     * @param translateX Robot relative X speed (% top speed)
     * @param translateY Robot relative Y speed (% top speed)
     * @param rotate Rotation speed (% top speed)
     */
    public void driveRobotRelative(double translateX, double translateY, double rotate) {
        // Create a chassisSpeeds object with the desired robot relative speeds
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(translateX * Drivebase.MAX_VELOCITY, translateY * Drivebase.MAX_VELOCITY, rotate * Drivebase.MAX_ROTATIONS_PER_SECOND * (Math.PI / 2));

        // Convert the robot relative speeds to swerve module states
        SwerveModuleState[] moduleStates = getSwerveDriveKinematics().toSwerveModuleStates(chassisSpeeds);

        // Set every swerve modules state to the desired state
        for (int module = 0; module < moduleStates.length; module++) {
            SwerveModule swerveModule = swerveModules[module];

            swerveModule.setState(moduleStates[module]);
        }
    }

    /**
     * Locks the swerve modules into the locked position
     */
    public void lockModules() {
        // Set the modules into the locked position
        // This will create a "X" pattern with the modules which will make the robot very difficult to rotate or move
        frontLeftModule.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backRightModule.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        frontRightModule.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        backLeftModule.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /**
     * Disables all the swerve module motors. Note this doesn't stop the motors or turn them to 0, this just stops moving them
     */
    public void stopModules() {
        // Turn off all module motors
        // Note this doesn't stop the motors or turn them to 0, this just stops moving them
        frontLeftModule.turnOffModule();
        frontRightModule.turnOffModule();
        backLeftModule.turnOffModule();
        backRightModule.turnOffModule();
    }

    /**
     * Get the robot azimuth
     * @return The robot's azimuth
     */
    public Rotation2d getGyroAngle() {
        // Get the gyro angle and convert it to a Rotation2d object
        return Rotation2d.fromDegrees(navX.getAngle() % 360);
    }

    /**
     * Get the robot's field relative speed
     * @return The robot's field relative speed
     */
    public FieldRelativeSpeed getFieldRelativeSpeed() {
        // Converts the module states to robot relative speeds
        ChassisSpeeds chassisSpeeds = swerveDriveKinematics.toChassisSpeeds(frontLeftModule.getState(), frontRightModule.getState(), backLeftModule.getState(), backRightModule.getState());

        // Convert the robot relative speeds to field relative speeds
        return new FieldRelativeSpeed(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, getGyroAngle());
    }

    /**
     * Get the robot's robot relative speed
     * @return The robot's robot relative speed
     */
    public ChassisSpeeds getRobotRelativeSpeed() {
        // Converts the module states to robot relative speeds
        return swerveDriveKinematics.toChassisSpeeds(frontLeftModule.getState(), frontRightModule.getState(), backLeftModule.getState(), backRightModule.getState());
    }

    /**
     * Get the robot's overall speed
     * @return The robot's speed in whatever direction it's going
     */
    public double getOverallSpeed() {
        // Get the robot's overall speed in whatever direction it's going (not velocity)

        // Converts the module states to robot relative speeds
        ChassisSpeeds robotCompSpeeds = swerveDriveKinematics.toChassisSpeeds(frontLeftModule.getState(), frontRightModule.getState(), backLeftModule.getState(), backRightModule.getState());

        // Use pythagorean theorem to convert robot relative speeds in the X and Y directions to a single speed
        return Math.sqrt(Math.pow(robotCompSpeeds.vxMetersPerSecond, 2) + Math.pow(robotCompSpeeds.vyMetersPerSecond, 2));
    }

    /**
     * Calculates the robot's current pose on the field based off of the limelight and the turret's angle
     * @return The robot's current pose on the field
     */
    public Pose2d calculatePoseFromLimelight() {
        // TODO: Make good description for this. I can explain it in person with paper but it's hard to get across through comments, i've tried
        double targetAbsAngle = getGyroAngle().getDegrees() + shooter.getTurretPosition() + limelight.getDeltaX();
        double targetDistance = shooter.getHubDistance();

        double fromZeroAngle = (180 - targetAbsAngle) % 360;

        double x = targetDistance * Math.cos(fromZeroAngle);
        double y = targetDistance * Math.sin(fromZeroAngle);

        Translation2d hubPosition = Shooter.HUB_POSITION;
        return new Pose2d(x + hubPosition.getX(), y + hubPosition.getY(), getGyroAngle());
    }

    /**
     * Gets the array of swerve modules
     * @return The array of swerve module objects
     */
    public SwerveModule[] getSwerveModules() {
        return swerveModules;
    }

    /**
     * Gets the swerve drive kinematics object
     * @return The swerve drive kinematics object for this robot
     */
    public SwerveDriveKinematics getSwerveDriveKinematics() {
        return swerveDriveKinematics;
    }
}
