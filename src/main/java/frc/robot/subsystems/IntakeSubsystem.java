package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;

public class IntakeSubsystem extends SubsystemBase{
    private SparkFlex intakeMotor;
    private SparkFlexConfig intakeMotorConfig;
    private PIDController pidController;

    public IntakeSubsystem () {
        intakeMotor = new SparkFlex(PortConstants.intakeMotorPort, MotorType.kBrushless);
        intakeMotorConfig = new SparkFlexConfig();

        intakeMotorConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake);
        
        intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pidController = new PIDController(0,0,0);
    }

}
