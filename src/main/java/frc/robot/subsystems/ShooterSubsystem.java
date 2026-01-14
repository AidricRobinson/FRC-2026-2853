package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;

public class ShooterSubsystem extends SubsystemBase {
  private SparkFlex leftFlywheel; 
  private SparkFlex rightFlywheel;
  private SparkFlexConfig leftFlywheelConfig;
  private SparkFlexConfig rightFlywheelConfig;
  double testSpeed = 0;

  
  public ShooterSubsystem() {
    leftFlywheel = new SparkFlex(PortConstants.leftFlywheelPort, MotorType.kBrushless);
    rightFlywheel = new SparkFlex(PortConstants.rightFlywheelPort, MotorType.kBrushless);

    leftFlywheelConfig = new SparkFlexConfig();
    leftFlywheelConfig
      .inverted(true)
      .idleMode(IdleMode.kCoast);

    leftFlywheel.configure(leftFlywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rightFlywheelConfig = new SparkFlexConfig();
    rightFlywheelConfig
      .inverted(false)
      .idleMode(IdleMode.kCoast);
     
    rightFlywheel.configure(rightFlywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setPower(double power) {
    leftFlywheel.set(power);
    rightFlywheel.set(power);
  }
  public void setLeftPower(double power){
    leftFlywheel.set(power);
  }
  public void setRightPower(double power){
    leftFlywheel.set(power);
  }


  public void shooterTestSpeedUp(){
    testSpeed += 0.1;
}
public void shooterTestSpeedDown(){
    testSpeed -= 0.1;
}
public void shooterTestSpeedShutdown(){
    testSpeed = 0;
}
public double getShooterTestSpeed(){
    return testSpeed;
}
public void setShooterTestPower(){
    setPower(testSpeed);
}
public void shutdown(){
    leftFlywheel.set(0);
    rightFlywheel.set(0);
}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ShooterTestSpeed", getShooterTestSpeed());
    SmartDashboard.updateValues();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
