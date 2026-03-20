package frc.robot.commands.VisionCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.GamepadConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;

public class AlignHubCommand extends Command {
    private final CommandSwerveDrivetrain swerve;
    private final LimelightSubsystem limelightSubsystem;
    private PIDController pidController;
    private GenericHID controller;
    private CommandXboxController joystick;

    // private double offset;
    
    // private double currentAngle;
    // private double translationalSpeed;
    private double output;
    private double min;
    public AlignHubCommand (CommandXboxController joystick, CommandSwerveDrivetrain swerve, LimelightSubsystem limelightSubsystem, GenericHID controller) {
        this.swerve = swerve;
        this.joystick = joystick;
        this.controller = controller;
        this.limelightSubsystem = limelightSubsystem;
        pidController = new PIDController(4, 5,0.1);

        output = 0;
    }

    @Override
    public void initialize() {
        pidController.setSetpoint(Constants.LimelightConstants.kAlignOffset);

    }

    @Override
    public void execute() {
        output = pidController.calculate(limelightSubsystem.getTx());
        min = Math.max(20, Math.abs(output));
        output = Math.copySign(min, output);
        ChassisSpeeds targetSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(0,0, output, Rotation2d.fromDegrees(swerve.getCurrentAngle()));
        
        swerve.setRotationalSpeed(output, targetSpeed);
        
        // translationalSpeed = pidController.calculate(currentAngle, offset);

        // ChassisSpeeds targetSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, translationalSpeed, Rotation2d.fromDegrees(currentAngle));
        // swerve.driveRobotRelative(targetSpeed);

        System.out.println("ALIGNMENT RUNNING! Error: " + pidController.getError());
        SmartDashboard.updateValues();
        periodic();
        
    }
    @Override
    public void end (boolean isFinished) {
        for (int i = 0; i < 10; i++) {
            System.out.println("ALIGNMENT COMMAND FINISHED");
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(pidController.getError()) < 0.5 || !controller.getRawButton(GamepadConstants.kLeftBumperPort);
    }

    
    public void periodic() {
        SmartDashboard.putNumber("DRIVE ALIGNMENT ERROR", pidController.getError());
        SmartDashboard.putNumber("DRIVE ALIGNMENT SETPOINT", pidController.getSetpoint());
        SmartDashboard.updateValues();
    }
}
