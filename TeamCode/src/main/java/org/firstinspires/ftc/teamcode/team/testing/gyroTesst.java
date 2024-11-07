package org.firstinspires.ftc.teamcode.team.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;

//@Disabled
@TeleOp
@Config
@Disabled
public class gyroTesst extends DarienOpModeAuto {

    public static int rotationAmount;

    public void runOpMode() {

        initControls();


        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("", imu.getRobotYawPitchRollAngles());
            print("", getRuntime());
//            if (gamepad1.a) {
//                AutoRotate(rotationAmount, 0.3, 0);
//            }
        }
    }

}


