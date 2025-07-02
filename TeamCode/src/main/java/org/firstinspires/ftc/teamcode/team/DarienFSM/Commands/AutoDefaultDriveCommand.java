package org.firstinspires.ftc.teamcode.team.DarienFSM.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.team.DarienFSM.Subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class AutoDefaultDriveCommand extends CommandBase {

    private DriveSubsystem driveSubsystem;


    public AutoDefaultDriveCommand(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() { // TODO SOLVERS
        driveSubsystem.autoUpdatePower();
    }

}
