package org.firstinspires.ftc.teamcode.team.DarienFSM;

import static org.firstinspires.ftc.teamcode.team.DarienHelperFunctions.initializeMotor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.team.DarienFSM.Commands.TeleDriveCommand;
import org.firstinspires.ftc.teamcode.team.DarienFSM.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.team.GoBildaPinpointDriver;


@Config
public abstract class DarienRobot extends CommandOpMode {

    //COMPONENTS
    private LynxModule controlHub;
    private GoBildaPinpointDriver odo;

    //DC MOTORS
    private MotorEx omniMotor0; // left front
    private MotorEx omniMotor1; // right front
    private MotorEx omniMotor2; // left rear
    private MotorEx omniMotor3; // right rear

    //SUBSYSTEMS
    public DriveSubsystem driveSubsystem;

    //CONTROLLERS
    GamepadEx drivePad;
    GamepadEx controlPad;

    //COMMANDS
    public TeleDriveCommand teleDriveCommand;

    //MISC
    boolean firstRun = true;


    ElapsedTime time = new ElapsedTime();

    public void initialize() {
        time.reset();

        controlHub = hardwareMap.get(LynxModule.class, "Control Hub"); // initalized the control hub
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL); // IDK what this does something about getting info from ch

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // allows us to push tele to both dh and dash

        configurePinpoint(); // initalizes the pinpoint sensor - just like with DarienOpMode

        //INITIALIZE THE DRIVE WHEELS
        omniMotor0 = initializeMotor("omniMotor0", hardwareMap);
        omniMotor1 = initializeMotor("omniMotor1", hardwareMap);
        omniMotor2 = initializeMotor("omniMotor2", hardwareMap);
        omniMotor3 = initializeMotor("omniMotor3", hardwareMap);

        omniMotor0.setInverted(true); //is direction to reverse
        omniMotor1.setInverted(false);
        omniMotor2.setInverted(false);
        omniMotor3.setInverted(true);

        //TODO potentially look at switching to FTCLib's prebaked mechanum drive - prob not but just a thought

        driveSubsystem = new DriveSubsystem(omniMotor0, omniMotor1, omniMotor2, omniMotor3, telemetry, odo, hardwareMap); //initializes drive so we can send commands
        register(driveSubsystem); // puts drive into the robot??? idk exactly


        //TODO add other subsystems, rn im just working on getting bare bones working

        drivePad = new GamepadEx(gamepad1); // initialize the controllers - the buttons are mapped elsewhere
        controlPad = new GamepadEx(gamepad2);

        configureCommands(); // key function where all the commands are initialized from the subsystems
    }

    @Override
    public void run() {
        if (firstRun) {
            super.run(); //whatever you need to run once
            firstRun = false;
        }

        super.run(); //IDK why this is here i dont think we need this and the one up top

        time.reset();
        //clear cache
        controlHub.clearBulkCache();
    }

    public void configureCommands() {
        //you never call subsystem commands directly. instead you call commands built in other classes that call the methods in the subsystems
        teleDriveCommand = new TeleDriveCommand(driveSubsystem, drivePad, drivePad::getLeftY, drivePad::getLeftX, drivePad::getRightX);

    }

    private void configurePinpoint() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        odo.setOffsets(0, 165); //these are tuned for 2/5/2025 robot

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
         */
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odo.setEncoderResolution(13.26291192);


        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD); //TODO check and fix these values


        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        //odo.recalibrateIMU();
        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Y offset", odo.getXOffset());
        telemetry.addData("X offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();
    }
}

