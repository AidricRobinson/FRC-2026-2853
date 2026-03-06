// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.YuanConstants;
import frc.robot.Constants.GamepadConstants;
import frc.robot.commands.AutonomousCommands.AutoIntakeCommand;
import frc.robot.commands.AutonomousCommands.AutoShootCommand;
import frc.robot.commands.OperatorCommands.AutoPivotDown;
import frc.robot.commands.OperatorCommands.AutoPivotUp;
import frc.robot.commands.OperatorCommands.DistanceShootCommand;
import frc.robot.commands.OperatorCommands.IntakeCommand;
import frc.robot.commands.OperatorCommands.LaunchFuelCommand;
import frc.robot.commands.OperatorCommands.ManualPivotDown;
import frc.robot.commands.OperatorCommands.ManualPivotUp;
import frc.robot.commands.OperatorCommands.SteepShootCommand;
import frc.robot.commands.TeleOpCommands.*;
import frc.robot.commands.TestCommands.HoodTestCommands.HoodTestDownCommand;
import frc.robot.commands.TestCommands.HoodTestCommands.HoodTestUpCommand;
import frc.robot.commands.TestCommands.IndexorTestCommands.*;
import frc.robot.commands.TestCommands.IntakeTestCommands.OutputTestCommands.IntakeTestOutputDown;
import frc.robot.commands.TestCommands.IntakeTestCommands.OutputTestCommands.IntakeTestOutputShutdown;
import frc.robot.commands.TestCommands.IntakeTestCommands.OutputTestCommands.IntakeTestOutputUp;
import frc.robot.commands.TestCommands.IntakeTestCommands.OutputTestCommands.IntakeTestSetOutput;
import frc.robot.commands.TestCommands.ShooterTestCommands.*;
import frc.robot.commands.VisionCommands.AlignHubCommand;
import frc.robot.subsystems.*;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import com.pathplanner.lib.auto.AutoBuilder;    
import com.pathplanner.lib.auto.NamedCommands;

//Imported Swerve drive 

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;


public class RobotContainer {
   


    //Swerve configuration?
    private double MaxSpeed = 1 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController joystick = new CommandXboxController(0);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final SendableChooser<Command> autoChooser;
    
    
    //Subsystems
    private final IndexorSubsystem m_IndexorSubsystem = new IndexorSubsystem();
    private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
    private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
    private final StorageSubsystem m_ConveyorSubsystem = new StorageSubsystem();
    private final LimelightSubsystem m_LimelightSubsystem = new LimelightSubsystem();
    private final PivotSubsystem m_PivotSubsystem = new PivotSubsystem();
    private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
    private final HoodSubsystem m_HoodSubsystem = new HoodSubsystem();
    private final Vision vision = new Vision(drivetrain);


    //Auto commands
    private final AutoShootCommand autoShootCommand4000RPM = new AutoShootCommand(m_ShooterSubsystem, m_ConveyorSubsystem, m_IndexorSubsystem, 7, 4000);
    private final AutoShootCommand autoShootCommand5000RPM = new AutoShootCommand(m_ShooterSubsystem, m_ConveyorSubsystem, m_IndexorSubsystem, 7, 5000);
    private final AutoIntakeCommand autoIntakeCommand = new AutoIntakeCommand(m_IntakeSubsystem);
    private final AutoPivotDown autoPivotDown = new AutoPivotDown(m_PivotSubsystem);


    //controllers
    private final GenericHID controller0 = new GenericHID(0);
    private final GenericHID controller1 = new GenericHID(1);
    private final GenericHID YuanCon = new GenericHID(1);

    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

        



  public RobotContainer() {
    //////////////////////////////////////////////////////////////////////////////////////////
    ///                            SECONDARY DRIVER                                        ///
    //////////////////////////////////////////////////////////////////////////////////////////

    // shoot
    // manual fire 
    // pivot up
    // pivot down
    // run intake
    // shooter angle 1
    // shooter angle 2
    // shooter angle 3
    // manual pivot up
    // manual pivot down



    
    //Autonomous booting up
    // NamedCommands.registerCommand("5000RPM Shoot Command", autoShootCommand5000RPM);
    // NamedCommands.registerCommand("4000RPM Shoot Command", autoShootCommand4000RPM);
    // NamedCommands.registerCommand("3000RPM Shoot Command",  new AutoShootCommand(m_ShooterSubsystem, m_ConveyorSubsystem, m_IndexorSubsystem, 7, 3000));
    // NamedCommands.registerCommand("Run Intake", autoIntakeCommand);


    // NamedCommands.registerCommand("PivotDown", autoPivotDown);
    
    autoChooser = AutoBuilder.buildAutoChooser("AAAAHHHH");
        SmartDashboard.putData("Auto Mode", autoChooser);



    configureBindings();
   
    FollowPathCommand.warmupCommand().schedule();
  }

  
  private void configureBindings() {
    ////////////////////////////////////////////////////////////////////////////////////////
    ///                             PRIMARY CONTROLLER SCHEME                            ///
    ////////////////////////////////////////////////////////////////////////////////////////
    
    // new JoystickButton(controller0, GamepadConstants.kRightBumperPort)
    //         .onTrue(new SwerveSlowModeCommand(drivetrain, controller0));
    // new JoystickButton(controller0, GamepadConstants.kLeftBumperPort)
    //         .onTrue(new AlignHubCommand(drivetrain, m_LimelightSubsystem, controller0));

    ////////////////////////////////////////////////////////////////////////////////////////
    ///                            SECONDARY CONTROLLER SCHEME                           ///
    ////////////////////////////////////////////////////////////////////////////////////////
     
    // new JoystickButton(YuanCon, YuanConstants.BT_A)
    //     .onTrue(new AutoPivotUp(m_PivotSubsystem));
    // new JoystickButton(YuanCon, YuanConstants.BT_B)
    //     .onTrue(new IntakeCommand(m_IntakeSubsystem, YuanCon));
    // new JoystickButton(YuanCon, YuanConstants.BT_C)
    //     .onTrue(new LaunchFuelCommand(m_ShooterSubsystem, m_ConveyorSubsystem, m_IndexorSubsystem, m_HoodSubsystem, YuanCon));
    // new JoystickButton(YuanCon, YuanConstants.BT_D)
    //     .onTrue(new AutoPivotDown(m_PivotSubsystem));
    // new JoystickButton(YuanCon, YuanConstants.BottomLeft)
    //     .onTrue(new ManualPivotUp(m_PivotSubsystem, YuanCon));
    // new JoystickButton(YuanCon, YuanConstants.BottomRight)
    //     .onTrue(new ManualPivotDown(m_PivotSubsystem, YuanCon));
    // new JoystickButton(YuanCon, YuanConstants.SideTop)
    //     .onTrue(new SteepShootCommand(m_ShooterSubsystem, m_ConveyorSubsystem, m_IndexorSubsystem, m_HoodSubsystem, m_LimelightSubsystem, YuanCon));
    // new JoystickButton(YuanCon, YuanConstants.SideBottom)
    //     .onTrue(new DistanceShootCommand(m_ShooterSubsystem, m_ConveyorSubsystem, m_IndexorSubsystem, m_HoodSubsystem, m_LimelightSubsystem, YuanCon));
   
    ////////////////////////////////////////////////////////////////////////////////////////
    ///                                TEST COMMANDS                                     ///
    ////////////////////////////////////////////////////////////////////////////////////////

    new JoystickButton(controller0, GamepadConstants.kYButtonPort)
        .onTrue(new ShooterTestSpeedUp(m_ShooterSubsystem, controller0));
    new JoystickButton(controller0, GamepadConstants.kXButtonPort)
        .onTrue(new ShooterTestSetSpeed(m_ShooterSubsystem, controller0));
    new JoystickButton(controller0, GamepadConstants.kBButtonPort)
        .onTrue(new ShooterTestShutdown(m_ShooterSubsystem, controller0));
    new JoystickButton(controller0, GamepadConstants.kAButtonPort)
        .onTrue(new ShooterTestSpeedDown(m_ShooterSubsystem, controller0));

    new POVButton(controller0, GamepadConstants.kDpadLeft)
        .onTrue(new IntakeTestSetOutput(m_IntakeSubsystem, controller0));
    new POVButton(controller0, GamepadConstants.kDpadUp)
        .onTrue(new IntakeTestOutputUp(m_IntakeSubsystem, controller0));
    new POVButton(controller0, GamepadConstants.kDpadDown)
        .onTrue(new IntakeTestOutputDown(m_IntakeSubsystem, controller0));
    new POVButton(controller0, GamepadConstants.kDpadRight)
        .onTrue(new IntakeTestOutputShutdown(m_IntakeSubsystem, controller0));

    new JoystickButton(controller0, GamepadConstants.kRightBumperPort)
        .onTrue(new StorageForwardCommand(m_ConveyorSubsystem, controller0));
    new JoystickButton(controller0, GamepadConstants.kRightBumperPort)
        .onTrue(new IndexorPID(m_IndexorSubsystem, controller0));

    new JoystickButton(controller1, GamepadConstants.kYButtonPort)
        .onTrue(new ManualPivotUp(m_PivotSubsystem, controller1));
    new JoystickButton(controller1, GamepadConstants.kAButtonPort)
        .onTrue(new ManualPivotDown(m_PivotSubsystem, controller1));

    new POVButton(controller1, GamepadConstants.kDpadUp)
        .onTrue(new HoodUpCommand(m_HoodSubsystem, controller1));
    new POVButton(controller1, GamepadConstants.kDpadDown)
        .onTrue(new HoodDownCommand(m_HoodSubsystem, controller1));


    ////////////////////////////////////////////////////////////////////////////////////////
    ///                                   SWERVE                                         ///
    /// ////////////////////////////////////////////////////////////////////////////////////
    
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.


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
        return autoChooser.getSelected();
  }
}