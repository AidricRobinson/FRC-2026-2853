package frc.robot.subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;



public class RotationSubsystem extends SubsystemBase{
    private TalonFX rotationMotor;
    private TalonFXConfiguration rotationMotorConfig;
    private PIDController pidController;

    private double testSpeed;
    private double setPoint;
    
    public RotationSubsystem() {
        rotationMotor = new TalonFX(PortConstants.rotationMotorPort);
        testSpeed = 0.2;
        pidController = new PIDController(0.005, 0, 0); // placeholders, though it is pretty close.

        // rotationMotorConfig = new TalonFXConfiguration();
        // rotationMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // rotationMotor.getConfigurator().apply(rotationMotorConfig);
        rotationMotor.setNeutralMode(NeutralModeValue.Brake);
        rotationMotor.setPosition(0);
    }


    public void setPower(double power){
        rotationMotor.set(power);
    }
    public double getEncoder(){
        return rotationMotor.getPosition().getValueAsDouble() * 2048;
        //2048 is the amt of counts per rotation
        //there is 65 degree range (starting at 15, ending at 80)
        //using counts, there is 126.03076 counts per degree
        //using rotations, ther is 0.06153846 rotations per degree
    }
    public double getRotationDegree(){
        return (rotationMotor.getPosition().getValueAsDouble() / 0.06153846) + 15;
        // rotations / 0.06153846 = A degree 0-65. Adding 15 allows you to get the real world degree
    }
    public double getDegreeOutput(){
        return pidController.calculate(getRotationDegree());
    }
    public void setPoint(double target){
        pidController.setSetpoint(target);
        setPoint = target;
    }
    public double getSetPoint(){
        return setPoint;
    }
    public double getPIDError(){
        return pidController.getError();
    }
    public double getOutput() {
        return pidController.calculate(getEncoder());
    }

    public double getShaftEncoder() {
        return getEncoder();
    }


    public void setMotorTestSpeed() {
        rotationMotor.set(testSpeed);
    }
    public void setMotorTestSpeedNeg() {
        rotationMotor.set(-testSpeed);
    }
    // Increments the pivot's speed by +5%
    public void upSpeed() {
        testSpeed += 0.05;
    }
    // Decrease the pivot's speed by -5%
    public void downSpeed() {
        testSpeed -= 0.05;
    }
    // Stop the motor
    public void testSpeedShutdown() {
        testSpeed = 0;
    }

    //SHUTDOWN METHODS
    public void shutdown () {
        rotationMotor.set(0);
    }
    @Override
    public void periodic(){
        SmartDashboard.putNumber("rotation test speed: ", testSpeed);
        SmartDashboard.putNumber("rotation Encoder: " , getEncoder());
        SmartDashboard.putNumber("rotation Setpoint", getSetPoint());
        SmartDashboard.putNumber("rotation Error", getPIDError());
        SmartDashboard.putNumber("rotation Output", getOutput());
        SmartDashboard.putNumber( "rotation Degrees", getRotationDegree());
        SmartDashboard.putNumber("rotation degree output", getDegreeOutput());
        SmartDashboard.updateValues();
    }
}
