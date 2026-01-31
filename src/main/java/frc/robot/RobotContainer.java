// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.GamepadConstants;
// import frc.robot.commands.ExampleCommand;
import frc.robot.commands.AutoCommands.Autos;
import frc.robot.commands.TeleOpCommands.IntakePID;
import frc.robot.commands.TeleOpCommands.ShooterCommand;
import frc.robot.commands.TeleOpCommands.ShooterPID;
import frc.robot.commands.TeleOpCommands.StorageBackwardCommand;
import frc.robot.commands.TeleOpCommands.StorageForwardCommand;
import frc.robot.commands.TeleOpCommands.StoragePID;
import frc.robot.commands.TestCommands.IntakeTestCommands.IntakeTestSetSpeed;
import frc.robot.commands.TestCommands.IntakeTestCommands.IntakeTestShutdown;
import frc.robot.commands.TestCommands.IntakeTestCommands.IntakeTestSpeedDown;
import frc.robot.commands.TestCommands.IntakeTestCommands.IntakeTestSpeedUp;
import frc.robot.commands.SwerveControlCommand;
import frc.robot.commands.SwerveControlCommand;
import frc.robot.commands.TestCommands.ShooterTestCommands.ShooterTestSetSpeed;
import frc.robot.commands.TestCommands.ShooterTestCommands.ShooterTestShutdown;
import frc.robot.commands.TestCommands.ShooterTestCommands.ShooterTestSpeedDown;
import frc.robot.commands.TestCommands.ShooterTestCommands.ShooterTestSpeedUp;
import frc.robot.commands.TestCommands.StorageTestCommands.StorageTestSetSpeed;
import frc.robot.commands.TestCommands.StorageTestCommands.StorageTestShutdown;
import frc.robot.commands.TestCommands.StorageTestCommands.StorageTestSpeedDown;
import frc.robot.commands.TestCommands.StorageTestCommands.StorageTestSpeedUp;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.SwerveModule;
import frc.robot.Constants.SwerveModuleConstants;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
// import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {

  private final ShuffleboardTab teleopTab = Shuffleboard.getTab("Teleop");
  private final ShuffleboardTab testTranPos = Shuffleboard.getTab("Test_Tran_Pos");
  private final ShuffleboardTab testTranVel = Shuffleboard.getTab("Test_Tran_Vel");
  private final ShuffleboardTab testRotPos = Shuffleboard.getTab("Test_Rot_Pos");
  private final ShuffleboardTab testRotVel = Shuffleboard.getTab("Test_Rot_Vel");
  private final ShuffleboardTab testPos = Shuffleboard.getTab("Test_Pos");
  private final ShuffleboardTab testGyroData = Shuffleboard.getTab("Test_Gyro_Data");
  private final CANBus canivore = new CANBus("drivetrain");

  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  private final StorageSubsystem m_ConveyorSubsystem = new StorageSubsystem();
   private final SwerveDriveSubsystem m_SwerveDriveSubsystem = new SwerveDriveSubsystem(    
    testTranPos,
    testTranVel,
    testRotPos,
    testRotVel, 
    testPos,
    testGyroData,
    canivore
  );
  private final GenericHID controller0 = new GenericHID(0);
  private final GenericHID controller1 = new GenericHID(1);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */



  public RobotContainer() {
    configureBindings();
       m_SwerveDriveSubsystem.setDefaultCommand(new SwerveControlCommand(
      m_SwerveDriveSubsystem,
      controller0
      )
    );

  }

  
  private void configureBindings() {

    //////////////////////////////////////////////////////////////////////////////////////////
    ///                               TEST COMMANDS                                        ///
    /// //////////////////////////////////////////////////////////////////////////////////////

    //Adjusting powers, currently not working for the time being :( ?
    //SHOOTER
    // new JoystickButton(controller0, GamepadConstants.kAButtonPort)
    //     .onTrue(new ShooterTestSpeedDown(m_ShooterSubsystem, controller0));
    // new JoystickButton(controller0, GamepadConstants.kXButtonPort)
    //     .onTrue(new ShooterTestSetSpeed(m_ShooterSubsystem, controller0));
    // new JoystickButton(controller0, GamepadConstants.kYButtonPort)
    //     .onTrue(new ShooterTestSpeedUp(m_ShooterSubsystem, controller0));
    // new JoystickButton(controller0, GamepadConstants.kBButtonPort)
    //     .onTrue(new ShooterTestShutdown(m_ShooterSubsystem, controller0));


    // new JoystickButton(controller0, GamepadConstants.kAButtonPort)
    //     .onTrue(new IntakeTestSpeedDown(m_IntakeSubsystem, controller0));
    // new JoystickButton(controller0, GamepadConstants.kXButtonPort)
    //     .onTrue(new IntakeTestSetSpeed(m_IntakeSubsystem, controller0));
    // new JoystickButton(controller0, GamepadConstants.kYButtonPort)
    //     .onTrue(new IntakeTestSpeedUp(m_IntakeSubsystem, controller0));
    // new JoystickButton(controller0, GamepadConstants.kBButtonPort)
    //     .onTrue(new IntakeTestShutdown(m_IntakeSubsystem, controller0));

    
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


    /// Stil needs to be tested, do not rely on this yet
    /// STORAGE
    // new JoystickButton(controller0, GamepadConstants.kLeftBumperPort)
    //         .onTrue(new StorageBackwardCommand(m_ConveyorSubsystem, controller0));
    // new JoystickButton(controller0, GamepadConstants.kRightBumperPort)
    //         .onTrue(new StorageForwardCommand(m_ConveyorSubsystem, controller0));
    // new JoystickButton(controller1, GamepadConstants.kRightBumperPort)
    //         .onTrue(new ShooterCommand(m_ShooterSubsystem, controller1));





    new JoystickButton(controller0, GamepadConstants.kAButtonPort)
            .onTrue(new ShooterPID(m_ShooterSubsystem, controller0));
    new JoystickButton(controller0, GamepadConstants.kYButtonPort)
            .onTrue(new StoragePID(m_ConveyorSubsystem, controller0));
      
  }  
  public Command getAutonomousCommand() {
    return Autos.exampleAuto(m_exampleSubsystem);


  }
}
