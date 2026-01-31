package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;

// import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
// import com.fasterxml.jackson.core.util.ReadConstrainedTextBuffer;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkFlexConfig;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;



public class SwerveModule {
    //TalonFX motor controllers
 private final TalonFX driveMotor;
 private final TalonFXConfiguration driveMotorConfig;
 private final TalonFX rotationMotor;
 private final TalonFXConfiguration rotationMotorConfig;

private final CANBus canivore;
 //PID controller
 private final PIDController rotationPID;
 //angle offsets
 private final CANcoder angleCANCoder;
 private final boolean CANCoderReversed;


 //constructor
 public SwerveModule(int drivePort, int rotationPort, boolean driveReversed, boolean rotationReversed,
    int CANCoderPort, double CANCoderOffset, boolean CANCoderReversed, CANBus canivore) {
      this.canivore = canivore;
      //initializing absolute encoder parameters 
     this.CANCoderReversed = CANCoderReversed;
     angleCANCoder = new CANcoder(CANCoderPort);
      
     // Configure the CANcoder for basic use
     CANcoderConfiguration configs = new CANcoderConfiguration();
     
     // This CANcoder should report absolute position from [-0.5, 0.5) rotations,
     // with a 0.26 rotation offset, with clockwise being positive
     configs.MagnetSensor
      .withAbsoluteSensorDiscontinuityPoint(SwerveModuleConstants.kAbsoluteSensorDiscontinuityPoint)
      .withMagnetOffset(CANCoderOffset)
      .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);
     
     // Write these configs to the CANcoder
     angleCANCoder.getConfigurator().apply(configs);
     
     //initializing TalonFX
     driveMotor = new TalonFX(drivePort, canivore);
     driveMotorConfig = new TalonFXConfiguration();
     

    //  driveMotorConfig
    //   .inverted(rotationReversed)
    //   .idleMode(IdleMode.kBrake)

     driveMotorConfig.MotorOutput.Inverted = driveReversed
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;
    driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

      
     //converting native units to measurements
    //   driveMotorConfig.encoder
    //   .positionConversionFactor(SwerveModuleConstants.kRotationToMeters)
    //   .velocityConversionFactor(SwerveModuleConstants.kMetersPerSecond);

      driveMotorConfig.Feedback.SensorToMechanismRatio = SwerveModuleConstants.kRotationToMeters;
      driveMotorConfig.Feedback.VelocityFilterTimeConstant = SwerveModuleConstants.kMetersPerSecond;
      driveMotor.getConfigurator().apply(driveMotorConfig);

    //   driveEncoder.getPosition();
    //   driveEncoder.getVelocity()

     rotationMotor = new TalonFX(rotationPort);
     rotationMotorConfig = new TalonFXConfiguration();

     rotationMotorConfig.MotorOutput.Inverted = driveReversed
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;
     rotationMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
     rotationMotorConfig.CurrentLimits.StatorCurrentLimit = SwerveModuleConstants.kRotationCurrentLimit;

     //rotationMotorConfig
     //   .inverted(rotationReversed)
     //   .idleMode(IdleMode.kBrake)
    
      //converting native units to measurements
      rotationMotorConfig.Feedback.SensorToMechanismRatio = SwerveModuleConstants.kRotationToMeters;
      rotationMotorConfig.Feedback.VelocityFilterTimeConstant = SwerveModuleConstants.kMetersPerSecond;
      rotationMotor.getConfigurator().apply(rotationMotorConfig);


     //setting motor configurations
    //  driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //  rotationMotor.configure(rotationMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


     //reseting faults 
     driveMotor.clearStickyFaults();
     rotationMotor.clearStickyFaults();

     //initializing encoders
     rotationMotor.getPosition().getValue();
       

     //initializing PID controllerimport com.revrobotics.spark.SparkFlex;

    
     rotationPID = new PIDController(
      SwerveModuleConstants.kTurningP, 
      SwerveModuleConstants.kTurningI, 
      SwerveModuleConstants.kTurningD
     );
     rotationPID.setTolerance(10);
       //calculates the least turning degrees to setpoint 
       //SwerveModuleConstants.kTurningTolerance
     rotationPID.enableContinuousInput(-Math.PI, Math.PI);

 }

 //=========================================================================== 
 // helper methods
 //===========================================================================

 /**
  * Returns the current position of the swerve module's drive motor
  * @return The current displacement of the drive motor in meters
  */
 
public double getError(){
  return rotationPID.getError();
}

 public double getDrivePosition() {
  return driveMotor.getPosition().getValueAsDouble();
 }
 /**
  * Returns the current position of the swerve module's rotation motor
  * @return The current rotation of the rotation motor in radians
  */
 public double getRotationPosition() {
  return rotationMotor.getPosition().getValueAsDouble();
 }

 /**
  * Returns the current velocity of the swerve module's drive motor
  * @return The current velocity of the drive motor in meters per second
  */
 public double getDriveVelocity() {
  return driveMotor.getVelocity().getValueAsDouble();
 }

 /**
  * Returns the current going into the drive motors
  * @return The current going into the drive motors in amps
  */
 public double getCurrentDrive() {
//   return driveMotor.getApppliedOutput();
     return driveMotor.get();
 }

 /**
  * Returns the current going into the rotation motors
  * @return The current going into the rotation motor in amps
  */
 public double getCurrentRotation() {
  return rotationMotor.get();
 }

 /**
  * Returns the current velocity of the swerve module's rotation motor
  * @return The current velocity of the rotation motor in radians per second
  */
 public double getRotationVelocity() {
  return rotationMotor.getVelocity().getValueAsDouble();
 }

 /**
  * Returns the current reading of the absolute encoder
  * Indicates which direction the motor should turn based on absoluteEncoderReversed 
  * @return The current reading of the absolute encoder in radians
  */
 public double getCANCoderReading() {
  double angle = (angleCANCoder.getAbsolutePosition().getValueAsDouble());
  return (angle * 2.0 * Math.PI) * (CANCoderReversed ? -1 : 1);   
 }

 public double getAbsoluteRotations() {
  return angleCANCoder.getAbsolutePosition().getValueAsDouble();
 }

 /**
  * Resets the encoders to their default position
  */
 

 public SwerveModuleState getModuleState() {
  return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromRadians(getCANCoderReading()));
 }
 /**
  * Stops the movement of the drive and rotation motor 
  */
 public void shutdown() {
  driveMotor.set(0);
  rotationMotor.set(0);
 }

 public void test(double power) {
  driveMotor.set(power);
 }
 public double getPIDSetpoint(){
  return rotationPID.getSetpoint();
 }
 public boolean InTolerance(){
  boolean InsideTolerance = rotationMotor.getPosition().getValueAsDouble() > rotationPID.getSetpoint() - 5 
                         && rotationMotor.getPosition().getValueAsDouble() < rotationPID.getSetpoint() + 5;
  return InsideTolerance;
 }
//  public boolean withinTollerance(){
//   rotationMotor.getPosition().getValueAsDouble() > 
//  }

 /**
  * Sets the optimal swerve module state to a given setpoint 
  * Changes the target drive and rotation speed of the module
  * @param currentState The swerve module state of the motor
  */
 public void setSwerveState(SwerveModuleState currentState) {
  if(Math.abs(currentState.speedMetersPerSecond) < 0.001) {
    shutdown();
    return;
  }
  
  // currentState = getModuleState();
  currentState.optimize(getModuleState().angle);
  double currentSpeed = currentState.speedMetersPerSecond;
  double limit = DriveConstants.kDriveMetersPerSecondLimit;
  double maximumSpeed = DriveConstants.kDriveMaxMetersPerSecond;

  driveMotor.set(Math.abs(currentSpeed) > limit
    ? Math.copySign(limit, currentSpeed) / maximumSpeed
    : currentSpeed / maximumSpeed);

  // driveMotor.set(1);
  rotationMotor.set(rotationPID.calculate(getCANCoderReading(), currentState.angle.getRadians()));
  SmartDashboard.putNumber("sybau", rotationPID.calculate(getCANCoderReading(), currentState.angle.getRadians()));
  SmartDashboard.putNumber("desired", getCANCoderReading());
  SmartDashboard.putNumber("current", currentState.angle.getRadians());
  SmartDashboard.updateValues(); 
 }
}