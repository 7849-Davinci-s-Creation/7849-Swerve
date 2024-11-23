package frc.robot;

import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            // Front Left
            new Translation2d(Constants.TRACKWIDTHMETERS / 2.0, Constants.TRACKWIDTHMETERS / 2.0),
            // Front Right
            new Translation2d(Constants.TRACKWIDTHMETERS / 2.0, -Constants.TRACKWIDTHMETERS / 2.0),
            // Back Left
            new Translation2d(-Constants.TRACKWIDTHMETERS / 2.0, Constants.TRACKWIDTHMETERS / 2.0),
            // Back Right
            new Translation2d(-Constants.TRACKWIDTHMETERS / 2.0, -Constants.TRACKWIDTHMETERS / 2.0));

    private final SwerveDrivePoseEstimator odometer = new SwerveDrivePoseEstimator(
            kinematics, getGyroAngle(), getModulePositions(), Constants.DRIVEODOMETRY_ORIGN);

    private final Rotation2d[] lastAngles = new Rotation2d[] {
            new Rotation2d(),
            new Rotation2d(),
            new Rotation2d(),
            new Rotation2d()
    };

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] desiredstates = kinematics
                .toSwerveModuleStates(ChassisSpeeds.discretize(chassisSpeeds, 0.02));

        double maxSpeed = Collections.max(Arrays.asList(desiredstates)).speedMetersPerSecond;

        if (maxSpeed <= Constants.DRIVEDEADBAND_MPS) {
            for (int i = 0; i < 4; i++) {
                stop();
            }
        } else {
            setModuleStates(desiredstates);
        }

    }

    public void stop() {
        for (int i = 0; i < 4; i++) {
            super.getModule(i).apply(new SwerveModuleState(0, lastAngles[i]), DriveRequestType.Velocity);
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        // MATH CRAP
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.MAXVELOCITY_MPS);

        for (int i = 0; i < 4; i++) {
            setModule(i, desiredStates[i]);
        }
    }

    public void setModule(int i, SwerveModuleState desiredState) {
        super.getModule(i).apply(desiredState, DriveRequestType.Velocity);
        lastAngles[i] = desiredState.angle;
    }

    public Pose2d getPose() {
        return odometer.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        odometer.resetPosition(getGyroAngle(), getModulePositions(), pose);
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public Rotation2d getGyroAngle() {
        return super.getPigeon2().getRotation2d();
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];

        for (int i = 0; i < 4; i++) {
            positions[i] = super.getModule(i).getPosition(false);
        }

        return positions;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];

        for (int i = 0; i < 4; i++) {
            states[i] = super.getModule(i).getCurrentState();
        }

        return states;
    }

    public void configDrivetrain() {
        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetPose,
                this::getChassisSpeeds,
                this::drive,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(5.0, 0, 0),

                        new PIDConstants(5.0, 0, 0),

                        1,

                        new Translation2d(Constants.TRACKWIDTHMETERS / 2.0, Constants.TRACKWIDTHMETERS / 2.0).getNorm(),

                        new ReplanningConfig()),

                () -> DriverStation.getAlliance().get().equals(Alliance.Red),

                this);
    }
}
