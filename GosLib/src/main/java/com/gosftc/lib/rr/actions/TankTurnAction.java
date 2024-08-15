package com.gosftc.lib.rr.actions;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.TankKinematics;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.gosftc.lib.rr.Drawing;
import com.gosftc.lib.rr.drive.TankDrive;
import com.gosftc.lib.rr.localizer.Pose2dState;
import com.gosftc.lib.rr.messages.DriveCommandMessage;
import com.gosftc.lib.rr.messages.PoseMessage;
import com.gosftc.lib.rr.messages.TankCommandMessage;
import com.gosftc.lib.rr.temp.TankDriveParams;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import java.util.function.Supplier;

public final class TankTurnAction implements Action {
    private static final TankDriveParams PARAMS = new TankDriveParams();

    private final TimeTurn m_turn;
    private final TankDrive m_drivetrain;
    private final TankKinematics m_kinematics;
    private final VoltageSensor m_voltageSensor;
    private final Supplier<Pose2dState> m_poseStateSupplier;

    private double m_beginTs = -1;

    public TankTurnAction(
            TimeTurn turn,
            TankDrive drive,
            TankKinematics kinematics,
            VoltageSensor voltageSensor,
            Supplier<Pose2dState> poseStateSupplier) {
        m_turn = turn;
        m_drivetrain = drive;
        m_kinematics = kinematics;
        m_voltageSensor = voltageSensor;

        m_poseStateSupplier = poseStateSupplier;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket p) {
        double t;
        if (m_beginTs < 0) {
            m_beginTs = Actions.now();
            t = 0;
        } else {
            t = Actions.now() - m_beginTs;
        }

        if (t >= m_turn.duration) {
            m_drivetrain.stop();
            return false;
        }

        Pose2dDual<Time> txWorldTarget = m_turn.get(t);
        m_drivetrain.publishTargetPose(new PoseMessage(txWorldTarget.value()));

        Pose2dState poseState = m_poseStateSupplier.get();
        Pose2d pose = poseState.m_pose;
        PoseVelocity2d robotVelRobot = poseState.m_velocity;

        PoseVelocity2dDual<Time> command =
                new PoseVelocity2dDual<>(
                        Vector2dDual.constant(new Vector2d(0, 0), 3),
                        txWorldTarget
                                .heading
                                .velocity()
                                .plus(
                                        PARAMS.turnGain * pose.heading.minus(txWorldTarget.heading.value())
                                                + PARAMS.turnVelGain
                                                        * (robotVelRobot.angVel - txWorldTarget.heading.velocity().value())));
        m_drivetrain.publishDriveCommand(new DriveCommandMessage(command));

        TankKinematics.WheelVelocities<Time> wheelVels = m_kinematics.inverse(command);
        double voltage = m_voltageSensor.getVoltage();
        final MotorFeedforward feedforward =
                new MotorFeedforward(PARAMS.kS, PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
        double leftPower = feedforward.compute(wheelVels.left) / voltage;
        double rightPower = feedforward.compute(wheelVels.right) / voltage;
        m_drivetrain.publishTankCommand(new TankCommandMessage(voltage, leftPower, rightPower));

        m_drivetrain.setMotorPower(leftPower, rightPower);

        Canvas c = p.fieldOverlay();
        // drawPoseHistory(c); // TODO(pj)

        c.setStroke("#4CAF50");
        Drawing.drawRobot(c, txWorldTarget.value());

        c.setStroke("#3F51B5");
        Drawing.drawRobot(c, pose);

        c.setStroke("#7C4DFFFF");
        c.fillCircle(m_turn.beginPose.position.x, m_turn.beginPose.position.y, 2);

        return true;
    }

    @Override
    public void preview(Canvas c) {
        c.setStroke("#7C4DFF7A");
        c.fillCircle(m_turn.beginPose.position.x, m_turn.beginPose.position.y, 2);
    }
}
