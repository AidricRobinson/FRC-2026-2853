package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants.PortConstants;


public class IntakeSubsystem extends SubsystemBase {
    private SparkFlex intakeMotor;
    private double testSpeed;
    private PIDController pidController;
    private SparkFlexConfig intakeMotorConfig;

    private double setPoint;

    public IntakeSubsystem(){
        intakeMotor = new SparkFlex(PortConstants.intakeMotorPort, MotorType.kBrushless);
        testSpeed = 0.2;
        pidController = new PIDController(0.0001, 0.00004, 1);

        intakeMotorConfig = new SparkFlexConfig();
        intakeMotorConfig
            .inverted(true)
            .idleMode(IdleMode.kCoast);

        intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    }
    public double getRPM() {
     return Math.abs(intakeMotor.getEncoder().getVelocity()); //be careful
    }
    public void setPower(double power){
        intakeMotor.set(power);
    }
    public void setMotorTestSpeed(){
        intakeMotor.set(testSpeed);
    }
    public void setMotorTestSpeedNeg(){
        intakeMotor.set(-testSpeed);
    }
    public void upSpeed(){
        testSpeed += 0.05;
    }
    public void downSpeed(){
        testSpeed -= 0.05;
    }
    public void testSpeedShutdown(){
        testSpeed = 0;
    }
    public double getIntakeTestSpeed(){
        return testSpeed;
    }
    public void shutdown(){
        intakeMotor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("IntakeTestSpeed", getIntakeTestSpeed());
        SmartDashboard.putNumber("IntakeRPM", getRPM());
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
