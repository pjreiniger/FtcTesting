package com.gosftc.lib.rr.localizer;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.gosftc.lib.rr.messages.MecanumLocalizerInputsMessage;
import com.gosftc.lib.rr.temp.MecanumParams;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class MecanumDriveLocalizer implements Localizer {
    private static final MecanumParams PARAMS = new MecanumParams();

    private final Encoder m_flEncoder;
    private final Encoder m_frEncoder;
    private final Encoder m_blEncoder;
    private final Encoder m_brEncoder;
    private final IMU m_imu;

    private final MecanumKinematics m_kinematics;

    private int m_flLastPos;
    private int m_frLastPos;
    private int m_blLastPos;
    private int m_brLastPos;

    private Rotation2d m_lastHeading;
    private boolean m_initialized;

        public MecanumDriveLocalizer(MecanumKinematics kinematics, Encoder fl, Encoder fr, Encoder bl, Encoder br, IMU imu) {
            m_flEncoder = fl;
            m_frEncoder = fr;
            m_blEncoder = bl;
            m_brEncoder = br;
            m_imu = imu;

            m_kinematics = kinematics;
        }

        @Override
        public Twist2dDual<Time> update() {
            PositionVelocityPair leftFrontPosVel = m_flEncoder.getPositionAndVelocity();
            PositionVelocityPair leftBackPosVel = m_blEncoder.getPositionAndVelocity();
            PositionVelocityPair rightBackPosVel = m_brEncoder.getPositionAndVelocity();
            PositionVelocityPair rightFrontPosVel = m_frEncoder.getPositionAndVelocity();

            YawPitchRollAngles angles = m_imu.getRobotYawPitchRollAngles();

            FlightRecorder.write("MECANUM_LOCALIZER_INPUTS", new MecanumLocalizerInputsMessage(
                    leftFrontPosVel, leftBackPosVel, rightBackPosVel, rightFrontPosVel, angles));

            Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));

            if (!m_initialized) {
                m_initialized = true;

                m_flLastPos = leftFrontPosVel.position;
                m_blLastPos = leftBackPosVel.position;
                m_brLastPos = rightBackPosVel.position;
                m_frLastPos = rightFrontPosVel.position;

                m_lastHeading = heading;

                return new Twist2dDual<>(
                        Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                        DualNum.constant(0.0, 2)
                );
            }

            double headingDelta = heading.minus(m_lastHeading);
            Twist2dDual<Time> twist = m_kinematics.forward(new MecanumKinematics.WheelIncrements<>(
                    new DualNum<Time>(new double[]{
                            (leftFrontPosVel.position - m_flLastPos),
                            leftFrontPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (leftBackPosVel.position - m_blLastPos),
                            leftBackPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (rightBackPosVel.position - m_brLastPos),
                            rightBackPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (rightFrontPosVel.position - m_frLastPos),
                            rightFrontPosVel.velocity,
                    }).times(PARAMS.inPerTick)
            ));

            m_flLastPos = leftFrontPosVel.position;
            m_blLastPos = leftBackPosVel.position;
            m_brLastPos = rightBackPosVel.position;
            m_frLastPos = rightFrontPosVel.position;

            m_lastHeading = heading;

            return new Twist2dDual<>(
                    twist.line,
                    DualNum.cons(headingDelta, twist.angle.drop(1))
            );
        }

    public List<Encoder> getLeftEncoders() {
        return Arrays.asList(m_flEncoder, m_blEncoder);
    }

    public List<Encoder> getRightEncoders() {
        return Arrays.asList(m_frEncoder, m_brEncoder);
    }

    public List<Encoder> getParEncoders() {
        return Collections.emptyList();
    }

    public List<Encoder> getPerpEncoders() {
        return Collections.emptyList();
    }
}
