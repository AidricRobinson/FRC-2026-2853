// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.GamepadConstants;
// import frc.robot.commands.ExampleCommand;
import frc.robot.commands.AutoCommands.Autos;
import frc.robot.commands.TeleOpCommands.IndexorBackwardCommand;
import frc.robot.commands.TeleOpCommands.IndexorForwardCommand;
import frc.robot.commands.TeleOpCommands.IndexorPID;
import frc.robot.commands.TeleOpCommands.ShooterCommand;
import frc.robot.commands.TeleOpCommands.ShooterPID;
import frc.robot.commands.TeleOpCommands.StorageBackwardCommand;
import frc.robot.commands.TeleOpCommands.StorageForwardCommand;
import frc.robot.commands.TeleOpCommands.StoragePID;
import frc.robot.commands.TeleOpCommands.SwerveSlowModeCommand;
import frc.robot.commands.TestCommands.IndexorTestCommands.IndexorTestSetSpeed;
import frc.robot.commands.TestCommands.IndexorTestCommands.IndexorTestShutdown;
import frc.robot.commands.TestCommands.IndexorTestCommands.IndexorTestSpeedDown;
import frc.robot.commands.TestCommands.IndexorTestCommands.IndexorTestSpeedUp;
import frc.robot.commands.TestCommands.IntakeTestCommands.IntakeTestSetRPM;
import frc.robot.commands.TestCommands.IntakeTestCommands.IntakeTestRPMShutdown;
import frc.robot.commands.TestCommands.IntakeTestCommands.IntakeTestRPMDown;
import frc.robot.commands.TestCommands.IntakeTestCommands.IntakeTestRPMUp;
import frc.robot.commands.TestCommands.IntakeTestCommands.OutputTestCommands.IntakeTestSetOutput;
import frc.robot.commands.TestCommands.IntakeTestCommands.OutputTestCommands.IntakeTestOutputShutdown;
import frc.robot.commands.TestCommands.IntakeTestCommands.OutputTestCommands.IntakeTestOutputDown;
import frc.robot.commands.TestCommands.IntakeTestCommands.OutputTestCommands.IntakeTestOutputUp;
import frc.robot.commands.TestCommands.PivotTestCommands.PivotTestSetSpeed;
import frc.robot.commands.TestCommands.PivotTestCommands.PivotTestShutdown;
import frc.robot.commands.TestCommands.PivotTestCommands.PivotTestSpeedDown;
import frc.robot.commands.TestCommands.PivotTestCommands.PivotTestSpeedUp;
import com.pathplanner.lib.auto.AutoBuilder;
// import frc.robot.commands.SwerveControlCommand;
// import frc.robot.commands.SwerveControlCommand;
import frc.robot.commands.TestCommands.ShooterTestCommands.ShooterTestSetSpeed;
import frc.robot.commands.TestCommands.ShooterTestCommands.ShooterTestShutdown;
import frc.robot.commands.TestCommands.ShooterTestCommands.ShooterTestSpeedDown;
import frc.robot.commands.TestCommands.ShooterTestCommands.ShooterTestSpeedUp;
import frc.robot.commands.TestCommands.StorageTestCommands.StorageTestSetSpeed;
import frc.robot.commands.TestCommands.StorageTestCommands.StorageTestShutdown;
import frc.robot.commands.TestCommands.StorageTestCommands.StorageTestSpeedDown;
import frc.robot.commands.TestCommands.StorageTestCommands.StorageTestSpeedUp;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IndexorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
// import frc.robot.subsystems.SwerveDriveSubsystem;
// import frc.robot.subsystems.SwerveModule;
import frc.robot.Constants.SwerveModuleConstants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
// import edu.wpi.first.wpilibj2.command.button.Trigger;

//Imported Swerve drive 

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class RobotContainer {
   
  private double MaxSpeed = 1 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private RobotConfig config;



    //////////////////////////////////////////////////////
    /// //////////////////////////////////////////////////


//     private final ShuffleboardTab teleopTab = Shuffleboard.getTab("Teleop");
//   private final ShuffleboardTab testTranPos = Shuffleboard.getTab("Test_Tran_Pos");
//   private final ShuffleboardTab testTranVel = Shuffleboard.getTab("Test_Tran_Vel");
//   private final ShuffleboardTab testRotPos = Shuffleboard.getTab("Test_Rot_Pos");
//   private final ShuffleboardTab testRotVel = Shuffleboard.getTab("Test_Rot_Vel");
//   private final ShuffleboardTab testPos = Shuffleboard.getTab("Test_Pos");
//   private final ShuffleboardTab testGyroData = Shuffleboard.getTab("Test_Gyro_Data");
//   private final CANBus canivore = new CANBus("drivetrain");

  private final IndexorSubsystem m_IndexorSubsystem = new IndexorSubsystem();
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  private final StorageSubsystem m_ConveyorSubsystem = new StorageSubsystem();
  private final LimelightSubsystem m_LimelightSubsystem = new LimelightSubsystem();
  private final PivotSubsystem m_PivotSubsystem = new PivotSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
//    private final SwerveDriveSubsystem m_SwerveDriveSubsystem = new SwerveDriveSubsystem(    
//     testTranPos,
//     testTranVel,
//     testRotPos,
//     testRotVel, 
//     testPos,
//     testGyroData,
//     canivore
//   );
  private final GenericHID controller0 = new GenericHID(0);
  private final GenericHID controller1 = new GenericHID(1);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
    

    private final SendableChooser<Command> autoChooser;


  public RobotContainer() {

    // Build an auto chooser. This will use Commands.none() as the default option.
    // As an example, this will only show autos that start with "comp" while at
    // competition as defined by the programmer

   configureBindings();
    //    m_SwerveDriveSubsystem.setDefaultCommand(new SwerveControlCommand(
    //   m_SwerveDriveSubsystem,
    //   controller0
    //   )
    // );
     try{
        config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
        // Handle exception as needed
        e.printStackTrace();
        }
    drivetrain.configurePathplanner(config);
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Path", autoChooser);

  }

  
  private void configureBindings() {

    //////////////////////////////////////////////////////////////////////////////////////////
    ///                               TEST COMMANDS                                        ///
    /// //////////////////////////////////////////////////////////////////////////////////////

    //Adjusting powers, currently not working for the time being :( ?
    //SHOOTER





    
    //STORAGE
    // new POVButton(controller0, GamepadConstants.kDpadRight)
    //    .onTrue(new StorageTestShutdown(m_ConveyorSubsystem, controller0));
    // new POVButton(controller0, GamepadConstants.kDpadLeft)
    //    .onTrue(new StorageTestSetSpeed(m_ConveyorSubsystem, controller0));
    // new POVButton(controller0, GamepadConstants.kDpadUp)
    //    .onTrue(new StorageTestSpeedUp(m_ConveyorSubsystem, controller0));
    // new POVButton(controller0, GamepadConstants.kDpadDown)
    //    .onTrue(new StorageTestSpeedDown(m_ConveyorSubsystem, controller0));
  
    //////////////////////////////////////////////////////////////////////////////////////////
    ///                              TELEOP COMMANDS                                       ///
    /// //////////////////////////////////////////////////////////////////////////////////////




   
    new JoystickButton(controller0, GamepadConstants.kAButtonPort)
        .onTrue(new ShooterTestSpeedDown(m_ShooterSubsystem, controller0));
    new JoystickButton(controller0, GamepadConstants.kXButtonPort)
        .onTrue(new ShooterTestSetSpeed(m_ShooterSubsystem, controller0));
    new JoystickButton(controller0, GamepadConstants.kYButtonPort)
        .onTrue(new ShooterTestSpeedUp(m_ShooterSubsystem, controller0));
    new JoystickButton(controller0, GamepadConstants.kBButtonPort)
        .onTrue(new ShooterTestShutdown(m_ShooterSubsystem, controller0)); 

    // new POVButton(controller0, GamepadConstants.kDpadDown)
    //     .onTrue(new StorageTestSpeedDown(m_ConveyorSubsystem, controller0));
    // new POVButton(controller0, GamepadConstants.kDpadLeft)
    //     .onTrue(new StorageTestSetSpeed(m_ConveyorSubsystem, controller0));
    // new POVButton(controller0, GamepadConstants.kDpadUp)
    //     .onTrue(new StorageTestSpeedUp(m_ConveyorSubsystem, controller0));
    // new POVButton(controller0, GamepadConstants.kDpadRight)
    //     .onTrue(new StorageTestShutdown(m_ConveyorSubsystem, controller0)); 

    
    new JoystickButton(controller0, GamepadConstants.kDpadDown)
        .onTrue(new PivotTestSpeedDown(m_PivotSubsystem, controller0));
    new POVButton(controller0, GamepadConstants.kDpadLeft)
        .onTrue(new PivotTestSetSpeed(m_PivotSubsystem, controller0));
    new POVButton(controller0, GamepadConstants.kDpadUp)
        .onTrue(new PivotTestSpeedUp(m_PivotSubsystem, controller0));
    new POVButton(controller0, GamepadConstants.kDpadRight)
        .onTrue(new PivotTestShutdown(m_PivotSubsystem, controller0)); 

    new JoystickButton(controller0, GamepadConstants.kLeftBumperPort)
        .onTrue(new StorageBackwardCommand(m_ConveyorSubsystem, controller0));
    new JoystickButton(controller0, GamepadConstants.kRightBumperPort)
        .onTrue(new StorageForwardCommand(m_ConveyorSubsystem, controller0));

    new JoystickButton(controller1, GamepadConstants.kLeftBumperPort)
        .onTrue(new IndexorBackwardCommand(m_IndexorSubsystem, controller1));
    new JoystickButton(controller1, GamepadConstants.kRightBumperPort)
        .onTrue(new IndexorForwardCommand(m_IndexorSubsystem, controller1));

    new JoystickButton(controller1, GamepadConstants.kAButtonPort)
        .onTrue(new IndexorTestSpeedDown(m_IndexorSubsystem, controller1));
    new JoystickButton(controller1, GamepadConstants.kXButtonPort)
        .onTrue(new IndexorTestSetSpeed(m_IndexorSubsystem, controller1));
    new JoystickButton(controller1, GamepadConstants.kYButtonPort)
        .onTrue(new IndexorTestSpeedUp(m_IndexorSubsystem, controller1));
    new JoystickButton(controller1, GamepadConstants.kBButtonPort)
        .onTrue(new IndexorTestShutdown(m_IndexorSubsystem, controller1));

    //////////////////////////////////////////////////////////////////////////////////////////
    ///                                RPM PID COMMANDS                                    ///
    /// //////////////////////////////////////////////////////////////////////////////////////
        
    // new POVButton(controller1, GamepadConstants.kDpadDown)
    //     .onTrue(new IntakeTestRPMDown(m_IntakeSubsystem, controller1));
    // new POVButton(controller1, GamepadConstants.kDpadLeft)
    //     .onTrue(new IntakeTestSetRPM(m_IntakeSubsystem, controller1));
    // new POVButton(controller1, GamepadConstants.kDpadUp)
    //     .onTrue(new IntakeTestRPMUp(m_IntakeSubsystem, controller1));
    // new POVButton(controller1, GamepadConstants.kDpadRight)
    //     .onTrue(new IntakeTestRPMShutdown(m_IntakeSubsystem, controller1)); 
    
    //////////////////////////////////////////////////////////////////////////////////////////
    ///                            SET SPEED COMMANDS                                      ///
    /// //////////////////////////////////////////////////////////////////////////////////////

    new POVButton(controller1, GamepadConstants.kDpadDown)
        .onTrue(new IntakeTestOutputDown(m_IntakeSubsystem, controller1));
    new POVButton(controller1, GamepadConstants.kDpadLeft)
        .onTrue(new IntakeTestSetOutput(m_IntakeSubsystem, controller1));
    new POVButton(controller1, GamepadConstants.kDpadUp)
        .onTrue(new IntakeTestOutputUp(m_IntakeSubsystem, controller1));
    new POVButton(controller1, GamepadConstants.kDpadRight)
        .onTrue(new IntakeTestOutputShutdown(m_IntakeSubsystem, controller1)); 














        System.err.println();



    ////////////////////////////////////////////////////////////////////////////////////////
    /// 
    /// ////////////////////////////////////////////////////////////////////////////////////
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
    new JoystickButton(controller0, GamepadConstants.kRightBumperPort)
            .onTrue(new SwerveSlowModeCommand(drivetrain, controller0));

        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * (CommandSwerveDrivetrain.getMaxSpeedThingy() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond))) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() *  (CommandSwerveDrivetrain.getMaxSpeedThingy() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond))) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }  
  public Command getAutonomousCommand() {
    // Simple drive forward auton
        // final var idle = new SwerveRequest.Idle();
        // return Commands.sequence(
        //     // Reset our field centric heading to match the robot
        //     // facing away from our alliance station wall (0 deg).
        //     drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
        //     // Then slowly drive forward (away from us) for 5 seconds.
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(0.5)
        //             .withVelocityY(0)
        //             .withRotationalRate(0)
        //     )
        //     .withTimeout(5.0),
        //     // Finally idle for the rest of auton
        //     drivetrain.applyRequest(() -> idle)
        // );
        return autoChooser.getSelected();
        // new PathPlannerAuto("AAAAHHHH.auto");
  }
}