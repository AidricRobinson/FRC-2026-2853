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
    private PIDController pidDown;
    private PIDController pidUp;
    private TalonFXConfiguration FlywheelConfig;
    double testSpeed = 0;
    double kFF = 0; //Placeholder
    private DutyCycleEncoder encoder;

    private double setPoint;
    public HoodSubsystem () {
        pidDown = pidConstants.hoodDownPID;
        pidUp = pidConstants.hoodUpPID;
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
        pidUp.setSetpoint(target);
        pidDown.setSetpoint(target);
        setPoint = target;
    }
    public double getUpError () {
        return pidUp.getError();
    }
    public double getDownError() {
        return pidDown.getError();
    }
    public double getSetpoint(){
        return pidUp.getSetpoint();
    }
    public void resetPID() {
        pidUp.reset();
        pidDown.reset();
    }
    public double getUpOutput () {
        pidUp.getD();
        return pidUp.calculate(getHoodAngle(), getSetpoint()) + kFF * pidUp.getSetpoint();
    }
    public double getDownOutput () {
        pidDown.getD();
        return pidDown.calculate(getHoodAngle(), getSetpoint()) + kFF * pidDown.getSetpoint();
    }
    public void updateError(){
        pidUp.getD();
        pidUp.calculate(getRPM(), getSetpoint());
        pidDown.getD();
        pidDown.calculate(getRPM(), getSetpoint());
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hood Test Speed", testSpeed);
        SmartDashboard.putNumber("Hood RPM", getRPM());
        SmartDashboard.putNumber("Hood SetPoint", setPoint);
        SmartDashboard.putNumber("Hood Encoder Angle", getHoodAngle());
        SmartDashboard.putNumber("Hood UP output", getUpOutput());
        SmartDashboard.putNumber("Hood DOWN output", getDownOutput());
        SmartDashboard.updateValues();
    }
    
}
