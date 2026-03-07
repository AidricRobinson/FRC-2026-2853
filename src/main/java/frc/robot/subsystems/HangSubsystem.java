package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class HangSubsystem {
    private SparkFlex lift; 
    private SparkFlexConfig liftConfig;
    private TalonFX pivot1;
    private TalonFXConfiguration pivot1Config;
    private TalonFX pivot2;   
    private TalonFXConfiguration pivot2Config;

    public HangSubsystem () {
        liftConfig = new SparkFlexConfig();
        liftConfig.idleMode(IdleMode.kBrake);
    }
}
