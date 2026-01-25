package frc.robot.commands.VisionCommands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AlignHubCommand extends Command {
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private LimelightSubsystem limelightSubsystem;

    private PIDController pidController;
    private double offset;
    
    private double currentAngle;
    private double translationalSpeed;

    public AlignHubCommand (SwerveDriveSubsystem swerveDriveSubsystem, LimelightSubsystem limelightSubsystem) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.limelightSubsystem = limelightSubsystem;  
        pidController = new PIDController(0, 0,0);

        addRequirements(swerveDriveSubsystem, limelightSubsystem);
    }

    @Override
    public void initialize() {
        offset = Math.toRadians(0); // needs to be changed
    }

    @Override
    public void execute() {
        currentAngle = Math.toRadians(limelightSubsystem.getTx());
        translationalSpeed = pidController.calculate(currentAngle, offset) * (-3);

        ChassisSpeeds targetSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(0, translationalSpeed, 0, Rotation2d.fromDegrees(currentAngle));
        // SwerveModuleState[] moduleStates = SwerveModuleConstants.kinematics.toSwerveModuleStates(targetSpeed);
        // swerveDriveSubsystem.setModuleStates(moduleStates);

        
    }
    @Override
    public void end (boolean isFinished) {
        System.out.println("Command end");
        swerveDriveSubsystem.shutdown();
    }
}
