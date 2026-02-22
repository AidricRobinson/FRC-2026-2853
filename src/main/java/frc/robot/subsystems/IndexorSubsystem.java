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


public class IndexorSubsystem extends SubsystemBase {
    private SparkFlex indexor;
    private double testRPM;
    private PIDController pidController;
    private SparkFlexConfig indexorMotorConfig;

    private double setPoint;

    public IndexorSubsystem(){
        indexor = new SparkFlex(PortConstants.indexorMotorPort, MotorType.kBrushless);
        testRPM = 0;
        pidController = pidConstants.indexorPID;

        indexorMotorConfig = new SparkFlexConfig();
        indexorMotorConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast);

        indexor.configure(indexorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    }
    public double getRPM() {
     return indexor.getEncoder().getVelocity(); //be careful
    }
    public void setPower(double power){
        indexor.set(power);
    }
    public void setMotorTestSpeed(){
        indexor.set(testRPM);
    }
    public void setMotorTestSpeedNeg(){
        indexor.set(-testRPM);
    }
    public void upSpeed(){
        testRPM += 250;
    }
    public void downSpeed(){
        testRPM -= 250;
    }
    public void testSpeedShutdown(){
        testRPM = 0;
    }
    public double getTestSpeed(){
        return testRPM;
    }
    public void shutdown(){
        indexor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Indexor Test Speed", getTestSpeed());
        SmartDashboard.putNumber("Indexor RPM", getRPM());
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
