package frc.robot;




///////////////////////////////////////////////////////////////////////////////
///                             Depreciated                                 ///
///////////////////////////////////////////////////////////////////////////////

public class PID {
    private double kP;
    private double kI;
    private double kD;
    private double kFF;
    private double integral;
    private double derivative;
    private double error;
    private double previousError = 0;
    private double output;
    private double processVariable;
    private double setPoint;

    public PID(double kP, double kI, double kD, double kFF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kFF = kFF;

        integral = 0;
    }

    public void setProcessVariable(double processVariable) {
        this.processVariable = processVariable;
    } 
    
    public void createSetPoint(double setPoint) {
        this.setPoint = setPoint;
    }

    public double getError() {
        previousError = error;
        error = setPoint - processVariable;
        return error;
    }
    public double getOutput() {
        error = getError();
        integral += error;
        derivative = (error - previousError) / 0.02;

        output = kP * error + kI  * integral + kD * derivative + Math.copySign(kFF, error);
        return output;
    }
}
