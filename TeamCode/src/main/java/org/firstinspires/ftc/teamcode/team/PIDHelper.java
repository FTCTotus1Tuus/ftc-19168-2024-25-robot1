package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PIDHelper {
    public double lastError;
    public double timeStep;
    public double integralAccum;
public static    double Kp = 0;
public static     double Ki = 0;
public static    double Kd = 0;

    public double PIDreturnCorrection(double error, double deltaTime) {

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
