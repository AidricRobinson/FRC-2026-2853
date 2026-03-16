package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.pidConstants;

public class HoodSubsystem extends SubsystemBase {
    private TalonFX angleMotor;
    private PIDController pidController;
    private TalonFXConfiguration FlywheelConfig;
    double testSpeed = 0;
    double kFF = 0; //Placeholder
    private DutyCycleEncoder encoder;

    private double setPoint;
    public HoodSubsystem () {
        pidController = pidConstants.hoodPID;
        angleMotor = new TalonFX(PortConstants.hoodMotor); //Placeholder
        angleMotor.setNeutralMode(NeutralModeValue.Brake);
        encoder = new DutyCycleEncoder(0,1,-0.66);
        encoder.setInverted(true);
        
    }


    //Test speeds
    public void hoodTestSpeedUp(){
        testSpeed += 1;
    }
    public void hoodTestSpeedDown(){
        testSpeed -= 1;
    }
    public double getTestRPM() {
        return testSpeed;
    }
    public void hoodTestSpeedShutdown(){
        testSpeed = 0;
    }
    public void setHoodTestPower(){
        setPower(testSpeed);
    }
    public double getHoodTestSpeed(){
        return testSpeed;
    }
    public double getHoodAngle(){
        return encoder.get();
    }

    public double getRPM() {
        return Math.abs(angleMotor.getVelocity().getValueAsDouble()); //be careful
    }
     public void setPower(double power){
        angleMotor.set(power);
    }
    public void shutdown(){
        angleMotor.set(0);
    }


    //PID methods
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


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hood Test Speed", testSpeed);
        SmartDashboard.putNumber("Hood RPM", getRPM());
        SmartDashboard.putNumber("Hood SetPoint", setPoint);
        SmartDashboard.putNumber("Hood Error", getError());
        SmartDashboard.putNumber("Hood Derivative", pidController.getD());
        SmartDashboard.putNumber("Hood Encoder Angle", getHoodAngle());
        SmartDashboard.updateValues();
    }
    
}
