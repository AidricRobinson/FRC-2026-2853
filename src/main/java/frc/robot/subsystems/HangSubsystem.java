package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PortConstants;

public class HangSubsystem extends SubsystemBase {
    private SparkFlex lift; 
    private SparkFlexConfig liftConfig;
    private TalonFX pivot1;
    
    private TalonFX pivot2;   

    public HangSubsystem () {
        liftConfig = new SparkFlexConfig();
        liftConfig.inverted(false).idleMode(IdleMode.kBrake);
        lift = new SparkFlex(PortConstants.hangMotorPort, MotorType.kBrushless);
        lift.configure(liftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pivot1 = new TalonFX(PortConstants.hangPivot1Port);
        pivot1.setNeutralMode(NeutralModeValue.Brake);

        pivot2 = new TalonFX(PortConstants.hangPivot2Port);
        pivot2.setNeutralMode(NeutralModeValue.Brake);
    }
    public void setLiftPower(double power) {
        lift.set(power);
    }
    public void shutdown() {
        lift.set(0);
        pivot1.set(0);
        pivot2.set(0);
    }
    public void setPivotPower(double power) {
        pivot1.set(power);
        pivot2.set(-power);
    }
    public double getLiftPosition() {
        return lift.getEncoder().getPosition();
    }
    public double getPivotPosition() {
        return pivot1.getPosition().getValueAsDouble();
    }
    
    @Override 
    public void periodic() {
        SmartDashboard.putNumber("HANG lift position", getLiftPosition());
        SmartDashboard.putNumber("HANG pivot position", getPivotPosition());
        SmartDashboard.updateValues();
    }
}
