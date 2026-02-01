package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;

public class PivotSubsystem extends SubsystemBase {
    private SparkFlex leftPivot;
    private SparkFlex rightPivot;
    private SparkFlexConfig leftConfig;
    private SparkFlexConfig rightConfig;
    private PIDController pidController;
    private double testOutput;

    public PivotSubsystem () {
        leftPivot = new SparkFlex(PortConstants.leftPivotPort, MotorType.kBrushless);
        rightPivot = new SparkFlex(PortConstants.rightPivotPort, MotorType.kBrushless);
        pidController = new PIDController(0, 0, 0);
        testOutput = 0;
        leftConfig = new SparkFlexConfig();
        rightConfig = new SparkFlexConfig();
        leftConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        rightConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        leftPivot.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightPivot.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public double getRotation() {
        return leftPivot.getEncoder().getPosition();
    }
    public void setPower(double power) {
        leftPivot.set(power);
        rightPivot.set(power);
    }
    public void shutdown() {
        leftPivot.set(0);
        rightPivot.set(0);
    }

    public void setTestOutput() {
        leftPivot.set(testOutput);
        rightPivot.set(testOutput);
    }
    public void increaseOutput() {
        testOutput += 0.05;
    }
    public void decreaseOutput() {
        testOutput -= 0.05;
    }
    public void testOutputShutdown () {
        testOutput = 0;
    }
    public double getTestOutput () {
        return testOutput;
    }

    public double getError() {
        return pidController.getError();
    }
    public double getOutput() {
        return pidController.calculate(getRotation(), getSetPoint());
    }
    public double getSetPoint() {
        return pidController.getSetpoint();
    }
    public void setPoint(double target) {
        pidController.setSetpoint(target);
    }
    public void updateError() {
        pidController.getD();
        pidController.calculate(getRotation(), getSetPoint());
    }
    public void resetPID() {
        pidController.reset();
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Test Output", testOutput);
        SmartDashboard.putNumber("Pivot Rotations", getRotation());
        SmartDashboard.putNumber("Pivot SetPoint", getSetPoint());
        SmartDashboard.putNumber("Pivot Error", getError());
        SmartDashboard.updateValues();
    }
    
}
