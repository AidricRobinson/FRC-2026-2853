// package frc.robot.subsystems;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj.Servo;
// import edu.wpi.first.wpilibj.Timer;

// public class FlywheelAngleSubsystem extends Servo {
//     private double speed;
//     private double length;
//     double lastTime = 0;

//     private double setPosition;
//     private double currentPosition;
//     public FlywheelAngleSubsystem (int channel, int length, int speed) {
//         super(channel);
//         setBoundsMicroseconds(1, 1, 1, 1, 1);
//         this.length = length;
//         this.speed = speed;
//     }

//     public void setPosition(double setpoint) {
//         setPosition = MathUtil.clamp(setpoint, 0, length);
//         setSpeed(setPosition / ((length * 2) - 1));
//     }

//     public void updateCurrentPosition() {
//         double dt = Timer.getFPGATimestamp() - lastTime;

//         if (currentPosition > setPosition + speed * dt) {
//             currentPosition -= speed * dt;
//         }
//         else if(currentPosition < setPosition - speed *dt) {
//             currentPosition += speed *dt;
//         }
//         else{
//             currentPosition = speed;
//         }
//     }

//     public double getCurrentPosition () {
//         return currentPosition;
//     }

//     public boolean isFinished () {
//         return currentPosition == setPosition;
//     }
// }
