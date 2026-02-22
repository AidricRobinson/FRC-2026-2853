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
import frc.robot.Constants.pidConstants;


public class IntakeSubsystem extends SubsystemBase {
    private SparkFlex intakeMotor;
    private double testRPM;
    private double testOutput;
    private PIDController pidController;
    private SparkFlexConfig intakeMotorConfig;

    private double setPoint;

    public IntakeSubsystem(){
        intakeMotor = new SparkFlex(PortConstants.intakeMotorPort, MotorType.kBrushless);
        testRPM = 0;
        testOutput = 0;
        pidController = pidConstants.intakePID;

        intakeMotorConfig = new SparkFlexConfig();
        intakeMotorConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    }
    public double getRPM() {
     return Math.abs(intakeMotor.getEncoder().getVelocity()); //be careful
    }
    public void setPower(double power){
        intakeMotor.set(power);
    }
    public void setMotorTestSpeed(){
        intakeMotor.set(testRPM);
    }
    public void setMotorTestSpeedNeg(){
        intakeMotor.set(-testRPM);
    }
    public void upRPM(){
        testRPM += 250;
    }
    public void downRPM(){
        testRPM -= 250;
    }
    public void testRPMShutdown(){
        testRPM = 0;
    }
    public double getIntakeTestRPM(){
        return testRPM;
    }


    public void upOutput(){
        testOutput += 0.05;
    }
    public void dowmOutput(){
        testOutput -= 0.05;
    }
    public void testOutputShutdown(){
        testOutput = 0;
    }
    public double getIntakeTestOutput(){
        return testOutput;
    }


    public void shutdown(){
        intakeMotor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Test Speed", getIntakeTestRPM());
        SmartDashboard.putNumber("Intake RPM", getRPM());
        SmartDashboard.putNumber("Intake Output", getIntakeTestOutput());
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
