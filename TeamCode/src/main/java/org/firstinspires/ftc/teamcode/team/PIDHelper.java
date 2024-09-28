package org.firstinspires.ftc.teamcode.team;

public class PIDHelper {
    public double lastError;
    public double timeStep;
    public double integralAccum;
    public double PIDreturnCorrection(double error, double deltaTime) {
        double Kp = 0;
        double Ki = 0;
        double Kd = 0;
        timeStep = deltaTime;

        double correction = Kp*error + Ki*integral(error) + Kd*derivative(error);
        lastError = error;

        return correction;
    }

    public double integral(double error) {
         integralAccum +=((lastError+error)/2) * timeStep;
        return integralAccum;
    }

    public double derivative(double error) {
        return (error-lastError) / (timeStep);
    }
}
