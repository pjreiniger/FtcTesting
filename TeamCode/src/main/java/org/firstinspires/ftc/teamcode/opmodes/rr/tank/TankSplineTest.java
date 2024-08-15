package org.firstinspires.ftc.teamcode.opmodes.rr.tank;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.MecanumChassis;
import org.firstinspires.ftc.teamcode.subsystems.TankDriveChassis;

public class TankSplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        TankDriveChassis drive = new TankDriveChassis(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineTo(new Vector2d(30, 30), Math.PI / 2)
                        .splineTo(new Vector2d(0, 60), Math.PI)
                        .build());
    }
}
