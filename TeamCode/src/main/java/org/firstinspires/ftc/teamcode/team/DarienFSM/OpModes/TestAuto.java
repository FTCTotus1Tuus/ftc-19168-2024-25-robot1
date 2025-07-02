package org.firstinspires.ftc.teamcode.team.DarienFSM.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.team.DarienFSM.Commands.AutoDefaultDriveCommand;
import org.firstinspires.ftc.teamcode.team.DarienFSM.DarienRobot;

@Autonomous
public class TestAuto extends DarienRobot {

    @Override
    public void initialize() {
        super.initialize();
        configure();

        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> driveSubsystem.setAutoDestination(0, 10, 0))
        ));
    }

    private void configure() {

        driveSubsystem.setDefaultCommand(autoDefaultDriveCommand);
    }

    @Override
    public void run() {
        super.run();
    }
}
