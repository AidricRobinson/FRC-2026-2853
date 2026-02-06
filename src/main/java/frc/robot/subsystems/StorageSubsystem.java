package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants.PortConstants;

public class StorageSubsystem extends SubsystemBase {
    private SparkFlex storageMotor;
    private double testSpeed;
    private SparkFlexConfig storageMotorConfig;
    private double setPoint;

    private PIDController pidController;
    public StorageSubsystem(){
        storageMotor = new SparkFlex(PortConstants.StorageMotorPort, MotorType.kBrushless);
        testSpeed= 0.2;
        storageMotorConfig = new SparkFlexConfig();
        storageMotorConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        storageMotor.configure(storageMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pidController = new PIDController(0.00005,0.000275,10);
    }
    public void setPower(double power){
        storageMotor.set(power);
    }
    public void setVoltage(double voltage) {
        storageMotor.setVoltage(voltage);
    }
    // public double ks() {
    //     return 1;
    // }
    // public double kv() {
    //     double radiansPerSecond = getRPM() * Math.PI / 30;
    //     return (getVoltage() - ks()) / radiansPerSecond;
    // }
    // public double ka() {
    //     return 1;
    // }
    public double getRPM() {
        return Math.abs(storageMotor.getEncoder().getVelocity());
    }
    public double getVoltage() {
        return storageMotor.getBusVoltage();
    }
    public void setMotorTestSpeed() {
        storageMotor.set(testSpeed);
    }
    public void setMotorTestSpeedNeg() {
        storageMotor.set(-testSpeed);
    }
    public void upSpeed() {
        testSpeed += 0.05;
    }
    public void downSpeed() {
        testSpeed -= 0.05;
    }
    public void testSpeedShutdown() {
        testSpeed = 0;
    }
    public double getTestSpeed(){
        return testSpeed;
    }
    public void shutdown(){
        storageMotor.set(0);
    }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Storage Test Speed", getTestSpeed());
    SmartDashboard.putNumber("Storage Error", getError());
    SmartDashboard.putNumber("Storage RPM", getRPM());
    SmartDashboard.putNumber("Storage SetPoint", getSetPoint());
    SmartDashboard.updateValues();
  }

  public double getError() {
    return pidController.getError();
  }
  public double getOutput() {
    return pidController.calculate(getRPM(), getSetPoint());
  }
  public double getSetPoint() {
    return pidController.getSetpoint();
  }
  public void setPoint(double target) {
    pidController.setSetpoint(target);
    setPoint = target;
  }
  public void updateError(){
    pidController.getD();
    pidController.calculate(getRPM(), getSetPoint());
  }
  public void reset() {
    pidController.reset();
  }

}