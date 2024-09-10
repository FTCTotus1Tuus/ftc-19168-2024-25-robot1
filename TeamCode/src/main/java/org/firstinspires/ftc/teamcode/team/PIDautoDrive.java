/*
Copyright 2021 FIRST Tech Challenge Team 0000

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Arrays;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.team.DarienOpMode;

import java.util.ArrayList;

@Config
@Autonomous
public class PIDautoDrive extends DarienOpMode {

    public static double encoderResolution = 537.7 ; //no change unless we change motors
    public static double wheelDiameter = 3.75; // inches
    public static double constMult = (wheelDiameter * (Math.PI));
    public static double constant = encoderResolution / constMult;
    public double lastTime;
    public double lastError;
    public double[] position;
    public DcMotor wheel0;
    public DcMotor wheel1;
    public DcMotor wheel2;
    public DcMotor wheel3;
    public IMU imu;
    public double timeStep;
    public double[] target;
    public double timeLeft;
    public ArrayList<double[]> trajectory;
    public static double x = 0;
    public static double y = 0;
    public static double rot = 0;
    double l = 6.8;
    double b = 5.8;
    double R = 2;

    TelemetryPacket tp;
    FtcDashboard dash;



    @Override
    public void runOpMode() {
        init_wheel();
        tp = new TelemetryPacket();
        dash = FtcDashboard.getInstance();
        waitForStart();
        while(opModeIsActive()) {
            setMotorPower(x, y, rot, 1);
        }

    }
    public void init_wheel(){
        wheel0 = hardwareMap.get(DcMotor.class, "omniMotor0");
        wheel1 = hardwareMap.get(DcMotor.class, "omniMotor1");
        wheel2 = hardwareMap.get(DcMotor.class, "omniMotor2");
        wheel3 = hardwareMap.get(DcMotor.class, "omniMotor3");

        wheel0.setDirection(DcMotorSimple.Direction.REVERSE);
        wheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        wheel0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheel3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                                RevHubOrientationOnRobot.UsbFacingDirection.DOWN
                        )
                )
        );
        imu.resetYaw();

    }
    public void setMotorPower (double x, double y, double rot, double speed){
        double adjX = x*Math.cos(getRawHeading()) - y*Math.sin(getRawHeading());
        double adjY = x*Math.sin(getRawHeading()) + y*Math.cos(getRawHeading());

        rot = (rot*(l/2 + b/2)) / (Math.sqrt(Math.pow(l/2,2)+Math.pow(b/2,2)));


        double motorPower0 = adjY+adjX+rot;
        double motorPower1 = adjY-adjX-rot;
        double motorPower2 = adjY-adjX+rot;
        double motorPower3 = adjY+adjX-rot;


        print("motor 0: ", motorPower0);
        tp.put("motor 0", motorPower0);
        dash.sendTelemetryPacket(tp);
//        double motorPower0 = speed * (y-x-(l+b) * rot) / R;
//        double motorPower2 = speed * (y+x-(l+b) * rot) / R;
//        double motorPower3 = speed * (y-x+(l+b) * rot) / R;
//        double motorPower1 = speed * (y+x+(l+b) * rot) / R;

        double[] scaledMotorPower = scalePower(motorPower0, motorPower1, motorPower2, motorPower3);

        wheel0.setPower(scaledMotorPower[0]);
        wheel1.setPower(scaledMotorPower[1]);
        wheel2.setPower(scaledMotorPower[2]);
        wheel3.setPower(scaledMotorPower[3]);

    }

    public double[] scalePower ( double motorPower0, double motorPower1, double motorPower2, double motorPower3)
    {
        double maxPower = Math.max(Math.max(Math.abs(motorPower0),Math.abs(motorPower1)),Math.max(Math.abs(motorPower2), Math.abs(motorPower3)));
        if (maxPower>1){
            motorPower0 /= maxPower;
            motorPower1 /= maxPower;
            motorPower2 /= maxPower;
            motorPower3 /= maxPower;
        }
        double[] returnPower = new double[]{
                motorPower0,motorPower1,motorPower2,motorPower3
    };
        return returnPower;
    }
    public double PIDreturnCorrection(double error) {
        double Kp = 0;
        double Ki = 0;
        double Kd = 0;

        return Kp*error + Ki*integral(error) + Kd*derivative(error);
    }

    public double integral(double error) {
        return ((lastError+error)/2) * timeStep;
    }

    public double derivative(double error) {
        return (error-lastError) / (timeStep);
    }

    public double relativePower ( double intended_power)
    {
        //makes sure the power going to the motors is constant over battery life
        return (13 * intended_power) / getVoltage();
    }



    public double getRawHeading () {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

}
