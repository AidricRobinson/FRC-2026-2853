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


    public HangSubsystem () {
        liftConfig = new SparkFlexConfig();
        liftConfig.inverted(false).idleMode(IdleMode.kBrake);
        lift = new SparkFlex(PortConstants.hangMotorPort, MotorType.kBrushless);
        lift.configure(liftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

       
    }
    public void setLiftPower(double power) {
        lift.set(power);
    }
    public void shutdown() {
        lift.set(0);

    }

    public double getHangPosition() {
        return lift.getEncoder().getPosition();
    }

    
    @Override 
    public void periodic() {
        SmartDashboard.putNumber("HANG encoder position", getHangPosition());
        SmartDashboard.updateValues();
    }
}
