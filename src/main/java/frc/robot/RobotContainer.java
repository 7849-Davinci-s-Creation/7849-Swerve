// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Playsog;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Candlesubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import lib.OperatorControllerUtil;

public class RobotContainer {
  // Subsystems
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final Candlesubsystem candlesubsystem = new Candlesubsystem(Constants.DeviceIDs.CANDLE_ID);

  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(Constants.OperatorConstants.JOYSTICK_PORT); // My
                                                                                                                       // joystick

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);
  private final SendableChooser<Command> automenu;

  public RobotContainer() {
    configureBindings();
    registerNamedCommands();

    drivetrain.configDrivetrain();
    automenu = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(automenu);
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive
                                                                                           // forward // with
            // negative Y (forward)
            .withVelocityY(-OperatorControllerUtil.handleDeadZone(joystick.getLeftX(), 0.05) * MaxSpeed) // Drive left
                                                                                                         // with
                                                                                                         // negative X
            // (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    joystick.back().onTrue(drivetrain.runOnce(() -> drivetrain.zeroRobotPose()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return automenu.getSelected();
  }

  public void registerNamedCommands() {
    NamedCommands.registerCommand("Around-The-World", new Playsog("aroundtheworld.chrp"));
    NamedCommands.registerCommand("sonic", new Playsog("sonic.chrp"));
  }


  public void robotInit() {
  }

 
  public void robotPeriodic() {
  }

  public void disabledInit() {
  }

  public void disabledPeriodic() {
  }

  public void disabledExit() {
  }

  public void autonomousInit() {
  }

  public void autonomousPeriodic() {
  }

  public void autonomousExit() {
  }

  public void teleopInit() {
  }

  public void teleopPeriodic() {
  }

  public void teleopExit() {
  }

  public void testInit() {
  }

  public void testPeriodic() {
  }

  public void testExit() {
  }

  public void simulationPeriodic() {
  }
}
