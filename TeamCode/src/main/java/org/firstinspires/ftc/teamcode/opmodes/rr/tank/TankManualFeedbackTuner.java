package org.firstinspires.ftc.teamcode.opmodes.rr.tank;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.gosftc.lib.rr.localizer.ThreeDeadWheelLocalizer;
import com.gosftc.lib.rr.localizer.TwoDeadWheelLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.MecanumChassis;
import org.firstinspires.ftc.teamcode.subsystems.TankDriveChassis;

public class TankManualFeedbackTuner extends LinearOpMode {
    public static double DISTANCE = 64;

    @Override
    public void runOpMode() {
        TankDriveChassis drive = new TankDriveChassis(hardwareMap, new Pose2d(0, 0, 0));

        if (drive.localizer instanceof TwoDeadWheelLocalizer) {
            if (TwoDeadWheelLocalizer.PARAMS.perpXTicks == 0 && TwoDeadWheelLocalizer.PARAMS.parYTicks == 0) {
                throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
            }
        } else if (drive.localizer instanceof ThreeDeadWheelLocalizer) {
            if (ThreeDeadWheelLocalizer.PARAMS.perpXTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par0YTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par1YTicks == 1) {
                throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
            }
        }
        waitForStart();

        while (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(0, 0, 0))
                            .lineToX(DISTANCE)
                            .lineToX(0)
                            .build());
        }
    }
}
