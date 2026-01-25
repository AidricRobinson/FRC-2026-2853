package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PID;
import frc.robot.Constants.PortConstants;

public class ShooterSubsystem extends SubsystemBase {
  private SparkFlex leftMotor; 
  private SparkFlex rightMotor;
  private PIDController pidController;
  private SparkFlexConfig leftMotorConfig;
  private SparkFlexConfig rightMotorConfig;
  double testSpeed = 0;

  private double setPoint;

  
  public ShooterSubsystem() {
    leftMotor = new SparkFlex(PortConstants.leftMotorPort, MotorType.kBrushless);
   rightMotor = new SparkFlex(PortConstants.rightMotorPort, MotorType.kBrushless);

   pidController = new PIDController(0.0001, 0.00004, 1);

    leftMotorConfig = new SparkFlexConfig();
    leftMotorConfig
      .inverted(true)
      .idleMode(IdleMode.kCoast);

    leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

   rightMotorConfig = new SparkFlexConfig();
   rightMotorConfig
      .inverted(false)
      .idleMode(IdleMode.kCoast);
     
   rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  public void setPower(double power) {
    leftMotor.set(power);
   rightMotor.set(power);
  }
  public void setLeftPower(double power){
    leftMotor.set(power);
  }
  public void setRightPower(double power){
    leftMotor.set(power);
  }
  public void shooterTestSpeedUp(){
    testSpeed += 200;
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
    return Math.abs(rightMotor.getEncoder().getVelocity()); //be careful
  }
  public double getShooterTestSpeed(){
    return testSpeed;
  }
  public void setShooterTestPower(){
    setPower(testSpeed);
  }
  public void shutdown(){
    leftMotor.set(0);
   rightMotor.set(0);


   
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ShooterTestSpeed", getShooterTestSpeed());
    SmartDashboard.putNumber("ShooterSliderTestingSpeed", testSpeed);
    SmartDashboard.putNumber("RPM of flywheel", getRPM());
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
    return pidController.calculate(getRPM(), getSetpoint());
  }
  public void updateError(){
    pidController.getD();
    pidController.calculate(getRPM(), getSetpoint());
  }

    
}
