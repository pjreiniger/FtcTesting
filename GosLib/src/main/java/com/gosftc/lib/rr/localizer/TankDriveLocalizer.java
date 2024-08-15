package com.gosftc.lib.rr.localizer;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.TankKinematics;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.gosftc.lib.rr.messages.TankLocalizerInputsMessage;
import com.gosftc.lib.rr.temp.TankDriveParams;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class TankDriveLocalizer implements Localizer {
    private static final TankDriveParams PARAMS = new TankDriveParams();

    private final List<Encoder> m_leftEncs;
    private final List<Encoder> m_rightEncs;
    private double m_lastLeftPos;
    private double m_lastRightPos;
    private boolean m_initialized;

    private final TankKinematics m_kinematics;

    public TankDriveLocalizer(
            TankKinematics kinematics, List<DcMotorEx> leftMotors, List<DcMotorEx> rightMotors) {
        m_kinematics = kinematics;
        {
            List<Encoder> leftEncs = new ArrayList<>();
            for (DcMotorEx m : leftMotors) {
                Encoder e = new OverflowEncoder(new RawEncoder(m));
                leftEncs.add(e);
            }
            this.m_leftEncs = Collections.unmodifiableList(leftEncs);
        }

        {
            List<Encoder> rightEncs = new ArrayList<>();
            for (DcMotorEx m : rightMotors) {
                Encoder e = new OverflowEncoder(new RawEncoder(m));
                rightEncs.add(e);
            }
            this.m_rightEncs = Collections.unmodifiableList(rightEncs);
        }

        // TODO: reverse encoder directions if needed
        //   leftEncs.get(0).setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public Twist2dDual<Time> update() {
        List<PositionVelocityPair> leftReadings = new ArrayList<>();
        List<PositionVelocityPair> rightReadings = new ArrayList<>();
        double meanLeftPos = 0.0;
        double meanLeftVel = 0.0;
        for (Encoder e : m_leftEncs) {
            PositionVelocityPair p = e.getPositionAndVelocity();
            meanLeftPos += p.position;
            meanLeftVel += p.velocity;
            leftReadings.add(p);
        }
        meanLeftPos /= m_leftEncs.size();
        meanLeftVel /= m_leftEncs.size();

        double meanRightPos = 0.0;
        double meanRightVel = 0.0;
        for (Encoder e : m_rightEncs) {
            PositionVelocityPair p = e.getPositionAndVelocity();
            meanRightPos += p.position;
            meanRightVel += p.velocity;
            rightReadings.add(p);
        }
        meanRightPos /= m_rightEncs.size();
        meanRightVel /= m_rightEncs.size();

        FlightRecorder.write(
                "TANK_LOCALIZER_INPUTS", new TankLocalizerInputsMessage(leftReadings, rightReadings));

        if (!m_initialized) {
            m_initialized = true;

            m_lastLeftPos = meanLeftPos;
            m_lastRightPos = meanRightPos;

            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2), DualNum.constant(0.0, 2));
        }

        TankKinematics.WheelIncrements<Time> twist =
                new TankKinematics.WheelIncrements<>(
                        new DualNum<Time>(new double[] {meanLeftPos - m_lastLeftPos, meanLeftVel})
                                .times(PARAMS.inPerTick),
                        new DualNum<Time>(
                                        new double[] {
                                            meanRightPos - m_lastRightPos, meanRightVel,
                                        })
                                .times(PARAMS.inPerTick));

        m_lastLeftPos = meanLeftPos;
        m_lastRightPos = meanRightPos;

        return m_kinematics.forward(twist);
    }

    public List<Encoder> getLeftEncoders() {
        return m_leftEncs;
    }

    public List<Encoder> getRightEncoders() {
        return m_rightEncs;
    }

    public List<Encoder> getParEncoders() {
        return Collections.emptyList();
    }

    public List<Encoder> getPerpEncoders() {
        return Collections.emptyList();
    }
}
