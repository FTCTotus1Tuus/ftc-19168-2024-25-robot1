package org.firstinspires.ftc.teamcode.team;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DarienOpModeAuto extends DarienOpMode {



    @Override
    public void initControls() {
        super.initControls();

        // reverse motors 2 and 3
        omniMotor2.setDirection(DcMotor.Direction.REVERSE);
        omniMotor3.setDirection(DcMotor.Direction.FORWARD);


    }




    public void moveXY(double x, double y, double power) {
        resetEncoder();

        int adjY = (int) Math.floor((y * inchesToEncoder + 0.5));
        int adjX = (int) Math.floor((x * inchesToEncoder + 0.5));

        omniMotor0.setTargetPosition(adjY+adjX);
        omniMotor1.setTargetPosition(adjY-adjX);
        omniMotor2.setTargetPosition(adjY-adjX);
        omniMotor3.setTargetPosition(adjY+adjX);

        setRunMode();
        setPower(power, adjX, adjY);
    }

    public void autoRotate(double targetPosRadians, double power) {
        //direction counter clockwise is -1 clockwise is 1
        double timeoutS = 3;
        rotationTolerance = PI/8;


        setToRotateRunMode();

        double error;
        boolean isRotating = true;
        double startTime = getRuntime();
        double direction = Math.signum(getErrorRot(targetPosRadians));
            setRotatePower(power, direction);
            while (isRotating) {
                error = getErrorRot(targetPosRadians);
                telemetry.addData("heading ", getRawHeading());
                telemetry.addData("error ", error);
                print("power", power);

                if (error<=rotationTolerance) {
                    isRotating = false;
                } else if (getRuntime() - startTime > timeoutS) {
                    isRotating = false;
                    telemetry.addData("timeout","");
                }
            }
        telemetry.addData("rotate end", "");
        telemetry.update();
        setRotatePower(0,0);
        resetEncoder();

    }

    public double sigmoid(double x) {
        //takes in any x value returns from (0,0) to (1,1) scale x accordingly
        return (2 / (1 + Math.pow(2.71, (-4 * x)))) - 1;
    }

    public double getVoltage() {
        return (hardwareMap.voltageSensor.iterator().next().getVoltage());
    }




    public void setRunMode() {
        omniMotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        omniMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        omniMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        omniMotor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setToRotateRunMode() {
        omniMotor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        omniMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        omniMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        omniMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPower(double power, double adjX, double adjY) {

        double[] motorPowers = scalePower((adjY + adjX), (adjY - adjX), (adjY - adjX), (adjY + adjX));

        omniMotor0.setPower(relativePower(power * motorPowers[0]));
        omniMotor1.setPower(relativePower(power * motorPowers[1]));
        omniMotor2.setPower(relativePower(power * motorPowers[2]));
        omniMotor3.setPower(relativePower(power * motorPowers[3]));
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


    public void setRotatePower(double power, double direction) {
        omniMotor0.setPower(relativePower(direction * power));
        omniMotor1.setPower(relativePower(-direction * power));
        omniMotor2.setPower(relativePower(-direction * power));
        omniMotor3.setPower(relativePower(direction * power));
    }

    public void resetEncoder() {
        omniMotor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        omniMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        omniMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        omniMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void waitForMotors() {
        while (omniMotor0.isBusy() && omniMotor1.isBusy() && omniMotor2.isBusy() && omniMotor3.isBusy()) {
        }
    }

    public double getErrorRot(double targetPosRot) {
        double errorBig = targetPosRot - getRawHeading();
        double errorSmol = targetPosRot - getRawHeading(false);

        return Math.min(Math.abs(errorSmol), Math.abs(errorBig));
    }


    public double getRawHeading (boolean convertToTwoPi) {


        double tempRot = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        if (convertToTwoPi) {
            if (tempRot < 0) {
                tempRot += Math.PI * 2;
            }
        }
        return tempRot;
    }

    public double getRawHeading() {
        return getRawHeading(true);
    }
}