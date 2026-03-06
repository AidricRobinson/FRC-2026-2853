package frc.robot.commands.VisionCommands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;

public class AlignHubCommand extends Command {
    private final CommandSwerveDrivetrain swerve;
    private LimelightSubsystem limelightSubsystem;

    private PIDController pidController;

    private GenericHID controller;
    // private double offset;
    
    // private double currentAngle;
    // private double translationalSpeed;
    private double output;

    public AlignHubCommand (CommandSwerveDrivetrain swerve, GenericHID controller) {
        this.swerve = swerve;
        this.controller = controller;
        pidController = new PIDController(0, 0,0);

        output = 0;
    }

    @Override
    public void initialize() {
        pidController.setSetpoint(swerve.getHubTurningAngle());

    }

    @Override
    public void execute() {
        output = pidController.calculate(swerve.getCurrentAngle());
        ChassisSpeeds targetSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, output, Rotation2d.fromDegrees(swerve.getCurrentAngle()));
        swerve.setRotationalSpeed(output, targetSpeed);
        // translationalSpeed = pidController.calculate(currentAngle, offset);

        // ChassisSpeeds targetSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, translationalSpeed, Rotation2d.fromDegrees(currentAngle));
        // swerve.driveRobotRelative(targetSpeed);

        
    }
    @Override
    public void end (boolean isFinished) {
        for (int i = 0; i < 10; i++) {
            System.out.println("ALIGNMENT COMMAND FINISHED");
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(pidController.getError()) < 0.1;
    }
}
