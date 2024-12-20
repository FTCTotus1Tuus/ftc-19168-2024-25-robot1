package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
public class PIDHelper extends LinearOpMode {
    public double lastError;
    public double timeStep;
    public double integralAccum;
    public static double Kp = 0.05;
    public static double Ki = 0.001;
    public static double Kd = 0;
    double correction = 0;

    public void runOpMode() {
    }

    PIDHelper(double p, double i, double d) {
        Kp = p; // Proportional coefficient = power to error
        Ki = i; // Integral coefficient = error over time
        Kd = d; // Derivative coefficient = change in error over time
    }

    public double PIDreturnCorrection(double error, double deltaTime) {

        timeStep = this.getRuntime();

        correction = Kp * error + Ki * integral(error) + Kd * derivative(error);
        lastError = error;

        return correction;
    }
//
//    public double getScalePIDSpeed(double speed, double errorX, double errorY, double errorH) {
//        PIDreturnCorrection()
//    }

    public double integral(double error) {
        integralAccum += ((lastError + error) / 2) * timeStep;
        return integralAccum;
    }

    public double derivative(double error) {
        return (error - lastError) / (timeStep);
    }
}
