package com.gosftc.lib.rr.actions;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.RamseteController;
import com.acmerobotics.roadrunner.TankKinematics;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.gosftc.lib.rr.Drawing;
import com.gosftc.lib.rr.drive.TankDrive;
import com.gosftc.lib.rr.localizer.Pose2dState;
import com.gosftc.lib.rr.messages.DriveCommandMessage;
import com.gosftc.lib.rr.messages.PoseMessage;
import com.gosftc.lib.rr.messages.TankCommandMessage;
import com.gosftc.lib.rr.temp.TankDriveParams;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import java.util.List;
import java.util.function.Supplier;

public final class TankFollowTrajectoryAction implements Action {
    private static final TankDriveParams PARAMS = new TankDriveParams();

    private final TimeTrajectory m_timeTrajectory;
    private final TankDrive m_drivetrain;
    private final TankKinematics m_kinematics;
    private final VoltageSensor m_voltageSensor;

    private final Supplier<Pose2dState> m_poseStateSupplier;

    private double m_beginTs = -1;

    private final double[] m_xPoints, m_yPoints;

    public TankFollowTrajectoryAction(
            TimeTrajectory t,
            TankDrive drive,
            TankKinematics kinematics,
            VoltageSensor voltageSensor,
            Supplier<Pose2dState> poseStateSupplier) {
        m_timeTrajectory = t;
        m_drivetrain = drive;
        m_kinematics = kinematics;
        m_voltageSensor = voltageSensor;
        m_poseStateSupplier = poseStateSupplier;

        List<Double> disps =
                com.acmerobotics.roadrunner.Math.range(
                        0, t.path.length(), Math.max(2, (int) Math.ceil(t.path.length() / 2)));
        m_xPoints = new double[disps.size()];
        m_yPoints = new double[disps.size()];
        for (int i = 0; i < disps.size(); i++) {
            Pose2d p = t.path.get(disps.get(i), 1).value();
            m_xPoints[i] = p.position.x;
            m_yPoints[i] = p.position.y;
        }
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

        if (t >= m_timeTrajectory.duration) {
            m_drivetrain.stop();

            return false;
        }

        DualNum<Time> x = m_timeTrajectory.profile.get(t);

        Pose2dDual<Arclength> txWorldTarget = m_timeTrajectory.path.get(x.value(), 3);
        m_drivetrain.publishTargetPose(new PoseMessage(txWorldTarget.value()));

        Pose2dState poseState = m_poseStateSupplier.get();
        Pose2d pose = poseState.m_pose;

        PoseVelocity2dDual<Time> command =
                new RamseteController(m_kinematics.trackWidth, PARAMS.ramseteZeta, PARAMS.ramseteBBar)
                        .compute(x, txWorldTarget, pose);
        m_drivetrain.publishDriveCommand(new DriveCommandMessage(command));

        TankKinematics.WheelVelocities<Time> wheelVels = m_kinematics.inverse(command);
        double voltage = m_voltageSensor.getVoltage();
        final MotorFeedforward feedforward =
                new MotorFeedforward(PARAMS.kS, PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
        double leftPower = feedforward.compute(wheelVels.left) / voltage;
        double rightPower = feedforward.compute(wheelVels.right) / voltage;
        m_drivetrain.publishTankCommand(new TankCommandMessage(voltage, leftPower, rightPower));

        m_drivetrain.setMotorPower(leftPower, rightPower);

        p.put("x", pose.position.x);
        p.put("y", pose.position.y);
        p.put("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

        Pose2d error = txWorldTarget.value().minusExp(pose);
        p.put("xError", error.position.x);
        p.put("yError", error.position.y);
        p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

        // only draw when active; only one drive action should be active at a time
        Canvas c = p.fieldOverlay();
        // drawPoseHistory(c); TODO(pj.reiniger)

        c.setStroke("#4CAF50");
        Drawing.drawRobot(c, txWorldTarget.value());

        c.setStroke("#3F51B5");
        Drawing.drawRobot(c, pose);

        c.setStroke("#4CAF50FF");
        c.setStrokeWidth(1);
        c.strokePolyline(m_xPoints, m_yPoints);

        return true;
    }

    @Override
    public void preview(Canvas c) {
        c.setStroke("#4CAF507A");
        c.setStrokeWidth(1);
        c.strokePolyline(m_xPoints, m_yPoints);
    }
}
