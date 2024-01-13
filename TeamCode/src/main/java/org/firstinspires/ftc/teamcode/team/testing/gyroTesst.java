package org.firstinspires.ftc.teamcode.team.testing;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
//@Disabled
@TeleOp
public class gyroTesst extends LinearOpMode {

    public void runOpMode() {


    IMU.Parameters parameters = new IMU.Parameters(

    new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));


    IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);

        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("", imu.getRobotYawPitchRollAngles());
            telemetry.update();
        }
}}
