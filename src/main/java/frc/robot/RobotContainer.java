// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.YuanConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.GamepadConstants;
import frc.robot.commands.AutomaticCommands.*;
import frc.robot.commands.OperatorCommands.AutoPivotDown;
import frc.robot.commands.OperatorCommands.*;
// import frc.robot.commands.AutonomousCommands.*;
// import frc.robot.commands.OperatorCommands.AutoPivotDown;
// import frc.robot.commands.OperatorCommands.AutoPivotUp;
// import frc.robot.commands.OperatorCommands.DistanceShootCommand;
import frc.robot.commands.OperatorCommands.IntakeCommand;
import frc.robot.commands.OperatorCommands.LaunchFuelCommand;
import frc.robot.commands.OperatorCommands.ManualPivotDown;
import frc.robot.commands.OperatorCommands.ManualPivotUp;
import frc.robot.commands.OperatorCommands.ShootCloseCommand;
import frc.robot.commands.OperatorCommands.SlowPivotUp;
import frc.robot.commands.OperatorCommands.SteepShootCommand;
import frc.robot.commands.VisionCommands.*;
import frc.robot.commands.TeleOpCommands.SwerveSlowModeCommand;
// import frc.robot.commands.OperatorCommands.SteepShootCommand;
import frc.robot.commands.TeleOpCommands.*;
import frc.robot.commands.TestCommands.HoodTestCommands.*;

import frc.robot.commands.TestCommands.IndexorTestCommands.*;
import frc.robot.commands.TestCommands.IntakeTestCommands.IntakeTestRPMDown;
import frc.robot.commands.TestCommands.IntakeTestCommands.IntakeTestRPMShutdown;
import frc.robot.commands.TestCommands.IntakeTestCommands.IntakeTestRPMUp;
import frc.robot.commands.TestCommands.IntakeTestCommands.IntakeTestSetRPM;
import frc.robot.commands.TestCommands.IntakeTestCommands.OutputTestCommands.IntakeTestOutputDown;
import frc.robot.commands.TestCommands.IntakeTestCommands.OutputTestCommands.IntakeTestOutputShutdown;
import frc.robot.commands.TestCommands.IntakeTestCommands.OutputTestCommands.IntakeTestOutputUp;
import frc.robot.commands.TestCommands.IntakeTestCommands.OutputTestCommands.IntakeTestSetOutput;
import frc.robot.commands.TestCommands.PivotTestCommands.PivotTestSetSpeed;
import frc.robot.commands.TestCommands.PivotTestCommands.PivotTestShutdown;
import frc.robot.commands.TestCommands.PivotTestCommands.PivotTestSpeedDown;
import frc.robot.commands.TestCommands.PivotTestCommands.PivotTestSpeedUp;
import frc.robot.commands.TestCommands.ShooterPIDCommands.ShooterTestRPMDown;
import frc.robot.commands.TestCommands.ShooterPIDCommands.ShooterTestRPMShutdown;
import frc.robot.commands.TestCommands.ShooterPIDCommands.ShooterTestRPMUp;
import frc.robot.commands.TestCommands.ShooterPIDCommands.ShooterTestSetRPM;
import frc.robot.commands.TestCommands.ShooterTestCommands.*;
// import frc.robot.commands.VisionCommands.AlignHubCommand;
import frc.robot.subsystems.*;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
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

import java.nio.file.ProviderNotFoundException;

import javax.naming.ldap.Control;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
    private Timer timer;
    
    
    //Subsystems
    private final IndexorSubsystem m_IndexorSubsystem = new IndexorSubsystem();
    private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
    private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
    private final LimelightSubsystem m_LimelightSubsystem = new LimelightSubsystem();
    private final PivotSubsystem m_PivotSubsystem = new PivotSubsystem();
    private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
    private final HoodSubsystem m_HoodSubsystem = new HoodSubsystem();
    // private final HangSubsystem hangSubsystem = new HangSubsystem();
    private final Vision vision = new Vision(drivetrain);



    //Auto commands
    private final AutoShootCommand autoShootCommand3000RPM = new AutoShootCommand(m_ShooterSubsystem, m_IndexorSubsystem, 5, 2250);
    private final AutoShootCommand autoShootCommand4000RPM = new AutoShootCommand(m_ShooterSubsystem, m_IndexorSubsystem, 5, 4000);
    private final AutoShootCommand autoShootCommand5000RPM = new AutoShootCommand(m_ShooterSubsystem, m_IndexorSubsystem, 5, 5000);
    // private final AutoIntakeCommand autoIntakeCommand = new AutoIntakeCommand(m_IntakeSubsystem);
    private final AutoScoreCommand autoScoreCommand = new AutoScoreCommand(m_ShooterSubsystem, m_IndexorSubsystem, m_LimelightSubsystem);
    private final AutoPivotDown autoPivotDown = new AutoPivotDown(m_PivotSubsystem);
    private final AutoIntakeCommand autoIntakeCommand = new AutoIntakeCommand(m_IntakeSubsystem, m_PivotSubsystem, 3);
    private final AutoLaunchCommand autoLaunchCommand = new AutoLaunchCommand(m_ShooterSubsystem, m_IndexorSubsystem, m_HoodSubsystem);
    //controllers
    private final GenericHID controller0 = new GenericHID(0);
    private final GenericHID controller1 = new GenericHID(1);
    private final GenericHID YuanCon = new GenericHID(2);

    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

        



  public RobotContainer() {
    timer = new Timer();

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
    NamedCommands.registerCommand("3000RPM", autoShootCommand3000RPM);
    NamedCommands.registerCommand("Launch", autoLaunchCommand);
    NamedCommands.registerCommand("AutoScore", autoScoreCommand);
    // NamedCommands.registerCommand("4000RPM Shoot Command", autoShootCommand4000RPM);
    // NamedCommands.registerCommand("3000RPM Shoot Command",  autoShootCommand5000RPM);
    // NamedCommands.registerCommand("Run Intake", new AutoIntakeCommand(m_IntakeSubsystem));
    NamedCommands.registerCommand("PivotDown", autoPivotDown);
    NamedCommands.registerCommand("RunIntake", autoIntakeCommand);
    
    autoChooser = AutoBuilder.buildAutoChooser("TestAutoShoot");
        SmartDashboard.putData("Auto Mode", autoChooser);



    configureBindings();
   
    FollowPathCommand.warmupCommand().schedule();
  }

  
  private void configureBindings() {
    ////////////////////////////////////////////////////////////////////////////////////////
    ///                             PRIMARY CONTROLLER SCHEME                            ///
    ////////////////////////////////////////////////////////////////////////////////////////
    
    new JoystickButton(controller0, GamepadConstants.kRightBumperPort)
            .onTrue(new SwerveSlowModeCommand(drivetrain, controller0));
    new JoystickButton(controller0, GamepadConstants.kLeftBumperPort)
            .onTrue(new AlignHubCommand(joystick, drivetrain, m_LimelightSubsystem, controller0));
    // new JoystickButton(controller0, GamepadConstants.kAButtonPort)
    //     .onTrue(new AlignBackwardCommand(drivetrain, controller0));
    // new JoystickButton(controller0, GamepadConstants.kBButtonPort)
    //     .onTrue(new AlignForwardCommand(drivetrain, controller0));


    ////////////////////////////////////////////////////////////////////////////////////////
    ///                            SECONDARY CONTROLLER SCHEME                           ///
    ////////////////////////////////////////////////////////////////////////////////////////
    /// 
    new JoystickButton(YuanCon, YuanConstants.BT_C)
        .onTrue(new LaunchFuelCommand(m_ShooterSubsystem, m_HoodSubsystem, YuanCon)
        .andThen(new AutoHoodDownCommand(m_HoodSubsystem)));
        
    new JoystickButton(YuanCon, YuanConstants.BT_B)
        .onTrue(new IntakeCommand(m_IndexorSubsystem, m_IntakeSubsystem, m_PivotSubsystem, YuanCon));
    new JoystickButton(YuanCon, YuanConstants.BT_A)
        .onTrue(new SteepShootCommand(m_ShooterSubsystem, m_HoodSubsystem, m_LimelightSubsystem, YuanCon)
        .andThen(new AutoHoodDownCommand(m_HoodSubsystem)));

    new JoystickButton(YuanCon, YuanConstants.BT_D)
        .onTrue(new ShootCloseCommand(m_ShooterSubsystem, YuanCon));
    //  new JoystickButton(YuanCon, YuanConstants.BT_D)
    //     .onTrue(new DistanceShootCommand(m_ShooterSubsystem, m_IndexorSubsystem, m_HoodSubsystem, m_LimelightSubsystem, YuanCon)
    //     .andThen(new AutoHoodDownCommand(m_HoodSubsystem)));
    new JoystickButton(YuanCon, YuanConstants.BottomLeft)
        .onTrue(new ManualPivotUp(m_PivotSubsystem, YuanCon));
    new JoystickButton(YuanCon, YuanConstants.BottomRight)
        .onTrue(new ManualPivotDown(m_PivotSubsystem, m_IntakeSubsystem, YuanCon));
    new JoystickButton(YuanCon, YuanConstants.SideTop)
        .onTrue(new IndexorForwardCommand(m_IndexorSubsystem, YuanCon));
    new JoystickButton(YuanCon, YuanConstants.SideBottom)
        .onTrue(new IndexorBackwardCommand(m_IndexorSubsystem, YuanCon));
    new JoystickButton(YuanCon, YuanConstants.BlueDiamond)
        .onTrue(new SlowPivotUp(m_PivotSubsystem, YuanCon));
   
    ////////////////////////////////////////////////////////////////////////////////////////
    ///                                TEST COMMANDS                                     ///
    ////////////////////////////////////////////////////////////////////////////////////////


        //POWER ADJUSTER

    // new JoystickButton(controller0, GamepadConstants.kYButtonPort)
    //     .onTrue(new ShooterTestSpeedUp(m_ShooterSubsystem, controller0));
    // new JoystickButton(controller0, GamepadConstants.kXButtonPort)
    //     .onTrue(new ShooterTestSetSpeed(m_ShooterSubsystem, controller0));
    // new JoystickButton(controller0, GamepadConstants.kBButtonPort)
    //     .onTrue(new ShooterTestShutdown(m_ShooterSubsystem, controller0));
    // new JoystickButton(controller0, GamepadConstants.kAButtonPort)
    //     .onTrue(new ShooterTestSpeedDown(m_ShooterSubsystem, controller0));


        //RPM ADJUSTER SHOOTER
        
    new JoystickButton(controller1, GamepadConstants.kYButtonPort)
        .onTrue(new ShooterTestRPMUp(m_ShooterSubsystem, controller1));
    new JoystickButton(controller1, GamepadConstants.kXButtonPort)
        .onTrue(new ShooterTestSetRPM(m_ShooterSubsystem, controller1));
    new JoystickButton(controller1, GamepadConstants.kBButtonPort)
        .onTrue(new ShooterTestRPMShutdown(m_ShooterSubsystem, controller1));
    new JoystickButton(controller1, GamepadConstants.kAButtonPort)
        .onTrue(new ShooterTestRPMDown(m_ShooterSubsystem, controller1));

        //HOOD TESTING

    // new POVButton(controller0, GamepadConstants.kDpadUp)
    //     .onTrue(new AutoHoodUpCommand(m_HoodSubsystem));
    // new POVButton(controller0, GamepadConstants.kDpadRight)
    //     .onTrue(new AutoHoodMiddleCommand(m_HoodSubsystem));
    // new POVButton(controller0, GamepadConstants.kDpadDown)
    //     .onTrue(new AutoHoodDownCommand(m_HoodSubsystem));

        //INTAKE TESTING

    // new POVButton(controller0, GamepadConstants.kDpadUp)
    //     .onTrue(new IntakeTestRPMUp(m_IntakeSubsystem, controller0));
    // new POVButton(controller0, GamepadConstants.kDpadLeft)
    //     .onTrue(new IntakeTestSetRPM(m_IntakeSubsystem, controller0));
    // new POVButton(controller0, GamepadConstants.kDpadRight)
    //     .onTrue(new IntakeTestRPMShutdown(m_IntakeSubsystem, controller0));
    // new POVButton(controller0, GamepadConstants.kDpadDown)
    //     .onTrue(new IntakeTestRPMDown(m_IntakeSubsystem, controller0));
        
    //     //INDEXOR TESTING
    // new JoystickButton(controller1, GamepadConstants.kRightBumperPort)
    //     .onTrue(new IndexorPID(m_IndexorSubsystem, controller1));
    // new JoystickButton(controller1, GamepadConstants.kLeftBumperPort)
    //     .onTrue(new IntakePID(m_IntakeSubsystem, controller1));
    // new JoystickButton(controller0, GamepadConstants.kLeftBumperPort)
    //     .onTrue(new IndexorBackwardCommand(m_IndexorSubsystem, controller0));

        //PIVOT

    // new JoystickButton(controller1, GamepadConstants.kYButtonPort)
    //     .onTrue(new PivotTestSpeedUp(m_PivotSubsystem, controller1));
    // new JoystickButton(controller1, GamepadConstants.kXButtonPort)
    //     .onTrue(new PivotTestSetSpeed(m_PivotSubsystem, controller1));
    // new JoystickButton(controller1, GamepadConstants.kBButtonPort)
    //     .onTrue(new PivotTestShutdown(m_PivotSubsystem, controller1));
    // new JoystickButton(controller1, GamepadConstants.kAButtonPort)
    //     .onTrue(new PivotTestSpeedDown(m_PivotSubsystem, controller1));

        //INTAKE?

    // new POVButton(controller1, GamepadConstants.kDpadUp)
    //     .onTrue(new IntakeTestOutputUp(m_IntakeSubsystem, controller1));
    // new POVButton(controller1, GamepadConstants.kDpadLeft)
    //     .onTrue(new IntakeTestSetOutput(m_IntakeSubsystem, controller1));
    // new POVButton(controller1, GamepadConstants.kDpadRight)
    //     .onTrue(new IntakeTestOutputShutdown(m_IntakeSubsystem, controller1));
    // new POVButton(controller1, GamepadConstants.kDpadDown)
    //     .onTrue(new IntakeTestOutputDown(m_IntakeSubsystem, controller1));

    // new JoystickButton(controller1, GamepadConstants.kRightBumperPort)
    //     .onTrue(new ManualPivotUp(m_PivotSubsystem, controller1));
    // new JoystickButton(controller1, GamepadConstants.kLeftBumperPort)
    //     .onTrue(new ManualPivotDown(m_PivotSubsystem, m_IntakeSubsystem, controller1));

    // new JoystickButton(controlle000000000oodSubsystem, controller1));

    // new POVButton(controller1, GamepadConstants.kDpadUp)
    //     .onTrue(new HangLiftUpCommand(hangSubsystem, controller1));
    // new POVButton(controller1, GamepadConstants.kDpadLeft)
    //     .onTrue(new HangLiftDownCommand(hangSubsystem, controller1));


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
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate * CommandSwerveDrivetrain.getMaxSpeedThingy()) // Drive counterclockwise with negative X (left)
            )
        );
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );





        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        // // Reset the field-centric heading on left bumper press.


        // joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        // drivetrain.registerTelemetry(logger::telemeterize);
    }  
    /// ScoreMiddle
    /// ScoreMiddleLeft
    /// ScoreMiddleRight
    /// ScoreMiddleOutpost
    /// 
    /// RightScoreMiddle
    /// RightMiddle
    /// 
    /// LeftMiddle
    /// LeftScoreMiddle


  public Command getAutonomousCommand() {
        String autoName = "ScoreMiddle";
        
        resetAutoPose(autoName);
         
        return new PathPlannerAuto(autoName);
        
        ///  CenterDepotScore
        /// LeftTwoCycle
        /// TestAutoShoot
        /// LeftIntakeAndDepot
        /// RightTwoCycle
    }
    // sets the pose of the robot based on the autonomous path name
    public void resetAutoPose(String autoName) {
        if (autoName.equals("ScoreMiddle") 
        || autoName.equals("ScoreMiddleLeft")
        || autoName.equals("ScoreMiddleRight")
        || autoName.equals("ScoreMiddleOutpost")) {
            drivetrain.resetPose(AutoConstants.middleAutoPose);
        }
        if (autoName.equals("LeftScoreMiddle")) {
            drivetrain.resetPose(AutoConstants.middleLeftPose);
        }
        if (autoName.equals("RightScoreMiddle")) {
            drivetrain.resetPose(AutoConstants.middleRightPose);
        }
        if (autoName.equals("LeftMiddle")) {
            drivetrain.resetPose(AutoConstants.leftAutoPose);
        }
        if (autoName.equals("RightMiddle")) {
            drivetrain.resetPose(AutoConstants.rightAutoPose);
        }
    }
}