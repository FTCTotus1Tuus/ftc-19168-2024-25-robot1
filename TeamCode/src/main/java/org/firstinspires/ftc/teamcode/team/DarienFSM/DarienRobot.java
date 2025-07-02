package org.firstinspires.ftc.teamcode.team.DarienFSM;

import static org.firstinspires.ftc.teamcode.team.DarienHelperFunctions.initializeMotor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.team.DarienFSM.Commands.AutoDefaultDriveCommand;
import org.firstinspires.ftc.teamcode.team.DarienFSM.Commands.TeleDriveCommand;
import org.firstinspires.ftc.teamcode.team.DarienFSM.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.team.GoBildaPinpointDriver;


@Config
public abstract class DarienRobot extends CommandOpMode {

    //COMPONENTS
    private LynxModule controlHub;

    //SUBSYSTEMS
    public DriveSubsystem driveSubsystem;

    //CONTROLLERS
    GamepadEx drivePad;
    GamepadEx controlPad;

    //COMMANDS
    public TeleDriveCommand teleDriveCommand;
    public AutoDefaultDriveCommand autoDefaultDriveCommand;

    //MISC
    boolean firstRun = true;


    ElapsedTime time = new ElapsedTime();

    public void initialize() { // TODO SOLVERS
        time.reset();

        controlHub = hardwareMap.get(LynxModule.class, "Control Hub"); // initalized the control hub
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL); // IDK what this does something about getting info from ch

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // allows us to push tele to both dh and dash


        //TODO potentially look at switching to FTCLib's prebaked mechanum drive - prob not but just a thought

        driveSubsystem = new DriveSubsystem("omniMotor0", "omniMotor1", "omniMotor2", "omniMotor3", telemetry, "odo", hardwareMap); //initializes drive so we can send commands
        register(driveSubsystem); // puts drive into the robot??? idk exactly


        //TODO add other subsystems, rn im just working on getting bare bones working

        drivePad = new GamepadEx(gamepad1); // initialize the controllers - the buttons are mapped elsewhere
        controlPad = new GamepadEx(gamepad2);

        configureCommands(); // key function where all the commands are initialized from the subsystems
    }

    @Override
    public void run() { // TODO SOLVERS
        if (firstRun) {
            super.run(); //whatever you need to run once
            firstRun = false;
        }

        super.run(); //IDK why this is here i dont think we need this and the one up top
        telemetry.update();
        time.reset();
        //clear cache
        controlHub.clearBulkCache();
    }

    public void configureCommands() { // TODO SOLVERS
        //you never call subsystem commands directly. instead you call commands built in other classes that call the methods in the subsystems
        teleDriveCommand = new TeleDriveCommand(driveSubsystem, drivePad, drivePad::getLeftY, drivePad::getLeftX, drivePad::getRightX);
        autoDefaultDriveCommand = new AutoDefaultDriveCommand(driveSubsystem);
    }

}

