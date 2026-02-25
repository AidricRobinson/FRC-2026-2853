package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GamepadConstants;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.pidConstants;

public class ShooterSubsystem extends SubsystemBase {
  private TalonFX FlywheelMotor;
  private PIDController pidController;
  private TalonFXConfiguration FlywheelConfig;
  // private SparkFlexConfig leftMotorConfig;
  // private SparkFlexConfig rightMotorConfig;
  double testSpeed = 0;

  private double setPoint;
  private double kFF = 0.00012;

  
  public ShooterSubsystem() {
    FlywheelMotor = new TalonFX(Constants.PortConstants.leftMotorPort);
    FlywheelMotor.setNeutralMode(NeutralModeValue.Coast);
    // FlywheelConfig = new TalonFXConfiguration();

  //  pidController = pidConstants.shooterPID;

  //   leftMotorConfig = new SparkFlexConfig();
  //   leftMotorConfig
  //     .inverted(true)
  //     .idleMode(IdleMode.kCoast);

  //   FlywheelMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  //  rightMotorConfig = new SparkFlexConfig();
  //  rightMotorConfig
  //     .inverted(false)
  //     .idleMode(IdleMode.kCoast);
     
  //  rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }
  public double calculateRPM(double tA) {
    return ((ShooterConstants.A * Math.pow(tA, 2))
    + (ShooterConstants.B * tA)
    + (ShooterConstants.C));

    // return 2000;
    
  }
  public void setPower(double power) {
    FlywheelMotor.set(power);
  //  rightMotor.set(power);
  }
  public void setLeftPower(double power){
    FlywheelMotor.set(power);
  }
  public void setRightPower(double power){
    FlywheelMotor.set(power);
  }
  public void shooterTestSpeedUp(){
    testSpeed += 250;
  }
  public void shooterTestSpeedDown(){
    testSpeed -= 50;
  }
  public double getTestRPM() {
    return testSpeed;
  }
  public void shooterTestSpeedShutdown(){
    testSpeed = 0;
  }

  public double getRPM() {
    return Math.abs(FlywheelMotor.getVelocity().getValueAsDouble()); //be careful
  }
  public double getShooterTestSpeed(){
    return testSpeed;
  }
  public void setShooterTestPower(){
    setPower(testSpeed);
  }
  public void shutdown(){
    FlywheelMotor.set(0);
  //  rightMotor.set(0);


   
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Test Speed", testSpeed);
    SmartDashboard.putNumber("Shooter RPM", getRPM());
    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("Error", getError());
    SmartDashboard.putNumber("Derivative", pidController.getD());
    SmartDashboard.updateValues();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setPoint(double target) {
    pidController.setSetpoint(target);
    setPoint = target;
  }

  public double getError () {
    return pidController.getError();
  }
  public double getSetpoint(){
    return pidController.getSetpoint();
  }

  public void resetPID() {
    pidController.reset();
  }

  public double getOutput () {
    pidController.getD();
    return pidController.calculate(getRPM(), getSetpoint()) + kFF * pidController.getSetpoint();
  }
  public void updateError(){
    pidController.getD();
    pidController.calculate(getRPM(), getSetpoint());
  }

    
}
