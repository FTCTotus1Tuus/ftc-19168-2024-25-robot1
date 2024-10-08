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

import java.util.Arrays;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

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
    PIDHelper PIDx;
    PIDHelper PIDy;
    public static double targetPosX = 1;
    public static double targetPosY = 1;
    public static double speed = 0.3;
    public List<Double> command;
        // command numbers
            // type, modifiers
            // movement: x y start time end time

    public List<List<Double>> commandList;


        // 0 = movement, 1 = wait, 2 = do action

    TelemetryPacket tp;
    FtcDashboard dash;



    @Override
    public void runOpMode() {
        commandList = new ArrayList<List<Double>>();

        init_wheel();
        tp = new TelemetryPacket();
        dash = FtcDashboard.getInstance();

        addCommand(0,100,0);
        addCommand(0,0,100);
        addCommand(0,-100,0);
        addCommand(0,0,-100);
        addCommand(0,-100,-100);
        addCommand(0,100,100);
        getNextCommand();
        waitForStart();

        startPIDMovement();

        while(opModeIsActive()) {

            setMotorPower(speed);
            //            setMotorPower(x, y, 1);
            timeStep = this.time - lastTime;
            lastTime = this.time;
        }
    }

    public void getNextCommand() {
            if (!commandList.isEmpty()) {
                commandList.remove(0);
            }
            command = commandList.get(0);
            if (command.get(0) == 0) {
                startPIDMovement();
                return;
            }
        }
    public void addCommand(double type, double x, double y){
        List<Double> temp = Arrays.asList(type, x, y);
        commandList.add(temp);
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

    public void startPIDMovement() {
        PIDx = new PIDHelper();
        PIDy = new PIDHelper();

        targetPosX = command.get(1) * constant;
        targetPosY = command.get(2) * constant;

        resetEncoderPositions();
    }
    public void setMotorPower (double speed){
        // questionable code
//        double adjX = x*Math.cos(getRawHeading()) - y*Math.sin(getRawHeading());
//        double adjY = x*Math.sin(getRawHeading()) + y*Math.cos(getRawHeading());
//
//        rot = (rot*(l/2 + b/2)) / (Math.sqrt(Math.pow(l/2,2)+Math.pow(b/2,2)));
        double errorX = getErrorX();
        double errorY = getErrorY();

        if (Math.abs(errorX) < 10 && Math.abs(errorY) < 10) {
            getNextCommand();
            return;
        }

        double adjX = PIDx.PIDreturnCorrection(errorX, timeStep) * speed;
        double adjY = PIDy.PIDreturnCorrection(errorY, timeStep) * speed;

        double motorPower0 = adjY+adjX;
        double motorPower1 = adjY-adjX;
        double motorPower2 = adjY-adjX;
        double motorPower3 = adjY+adjX;

        telemetry.addData("motor 0 encoder count: ", wheel0.getCurrentPosition());
        telemetry.addData("adjX: ", adjX);
        telemetry.addData("adjY: ", adjY);
        telemetry.addData("timestep: ", timeStep );
        telemetry.addData("current command: ", command);
        print("motor 0 power: ", motorPower0);

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

    public double getErrorX() {
        return targetPosX - (wheel0.getCurrentPosition() + wheel3.getCurrentPosition() - wheel1.getCurrentPosition() - wheel2.getCurrentPosition());
    }

    public double getErrorY() {
        return targetPosY - (wheel0.getCurrentPosition() + wheel3.getCurrentPosition() + wheel1.getCurrentPosition() + wheel2.getCurrentPosition());
    }

    void resetEncoderPositions() {
        wheel0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wheel0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


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


    public double relativePower ( double intended_power)
    {
        //makes sure the power going to the motors is constant over battery life
        return (13 * intended_power) / getVoltage();
    }



    public double getRawHeading () {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

}
