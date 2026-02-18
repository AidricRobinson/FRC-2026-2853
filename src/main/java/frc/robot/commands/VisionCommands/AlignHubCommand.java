package frc.robot.commands.VisionCommands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;

public class AlignHubCommand extends Command {
    private final CommandSwerveDrivetrain swerve;
    private LimelightSubsystem limelightSubsystem;

    private PIDController pidController;
    private double offset;
    
    private double currentAngle;
    private double translationalSpeed;

    public AlignHubCommand (CommandSwerveDrivetrain swerve, LimelightSubsystem limelightSubsystem) {
        this.swerve = swerve;
        this.limelightSubsystem = limelightSubsystem;  
        pidController = new PIDController(0, 0,0);
    }

    @Override
    public void initialize() {
        offset = Math.toRadians(0); // needs to be changed
    }

    @Override
    public void execute() {
        currentAngle = Math.toRadians(limelightSubsystem.getTx());
        translationalSpeed = pidController.calculate(currentAngle, offset);

        ChassisSpeeds targetSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, translationalSpeed, Rotation2d.fromDegrees(currentAngle));
        swerve.driveRobotRelative(targetSpeed);

        
    }
    @Override
    public void end (boolean isFinished) {
        for (int i = 0; i < 10; i++) {
            System.out.println("ALIGNMENT COMMAND FINISHED");
        }
    }

    // @Override
    // public boolean isFinished() {
    //     return Math.abs(pidController.getError()) < 0.1;
    // }
}
