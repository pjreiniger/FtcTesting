package com.gosftc.lib.rr.localizer;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.gosftc.lib.rr.messages.TwoDeadWheelInputsMessage;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import java.util.Collections;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Config
public final class TwoDeadWheelLocalizer implements Localizer {
    @SuppressWarnings({"PMD.RedundantFieldInitializer", "PMD.FieldNamingConventions"})
    public static class Params {
        public double parYTicks = 0.0; // y position of the parallel encoder (in tick units)
        public double perpXTicks = 0.0; // x position of the perpendicular encoder (in tick units)
    }

    public static final Params PARAMS = new Params();

    private final Encoder m_par;
    private final Encoder m_perp;
    private final IMU m_imu;

    private int m_lastParPos;
    private int m_lastPerpPos;
    private Rotation2d m_lastHeading;

    private final double m_inPerTick;

    private double m_lastRawHeadingVel;
    private double m_headingVelOffset;
    private boolean m_initialized;

    public TwoDeadWheelLocalizer(HardwareMap hardwareMap, IMU imu, double inPerTick) {
        // TODO: make sure your config has **motors** with these names (or change them)
        //   the encoders should be plugged into the slot matching the named motor
        //   see
        // https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        m_par = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "par")));
        m_perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "perp")));

        // TODO: reverse encoder directions if needed
        //   par.setDirection(DcMotorSimple.Direction.REVERSE);

        this.m_imu = imu;

        this.m_inPerTick = inPerTick;

        FlightRecorder.write("TWO_DEAD_WHEEL_PARAMS", PARAMS);
    }

    public Twist2dDual<Time> update() {
        PositionVelocityPair parPosVel = m_par.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = m_perp.getPositionAndVelocity();

        YawPitchRollAngles angles = m_imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = m_imu.getRobotAngularVelocity(AngleUnit.RADIANS);

        FlightRecorder.write(
                "TWO_DEAD_WHEEL_INPUTS",
                new TwoDeadWheelInputsMessage(parPosVel, perpPosVel, angles, angularVelocity));

        Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));

        // see https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/617
        double rawHeadingVel = angularVelocity.zRotationRate;
        if (Math.abs(rawHeadingVel - m_lastRawHeadingVel) > Math.PI) {
            m_headingVelOffset -= Math.signum(rawHeadingVel) * 2 * Math.PI;
        }
        m_lastRawHeadingVel = rawHeadingVel;

        if (!m_initialized) {
            m_initialized = true;

            m_lastParPos = parPosVel.position;
            m_lastPerpPos = perpPosVel.position;
            m_lastHeading = heading;

            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2), DualNum.constant(0.0, 2));
        }

        double headingVel = m_headingVelOffset + rawHeadingVel;

        int parPosDelta = parPosVel.position - m_lastParPos;
        int perpPosDelta = perpPosVel.position - m_lastPerpPos;
        double headingDelta = heading.minus(m_lastHeading);

        Twist2dDual<Time> twist =
                new Twist2dDual<>(
                        new Vector2dDual<>(
                                new DualNum<Time>(
                                                new double[] {
                                                    parPosDelta - PARAMS.parYTicks * headingDelta,
                                                    parPosVel.velocity - PARAMS.parYTicks * headingVel,
                                                })
                                        .times(m_inPerTick),
                                new DualNum<Time>(
                                                new double[] {
                                                    perpPosDelta - PARAMS.perpXTicks * headingDelta,
                                                    perpPosVel.velocity - PARAMS.perpXTicks * headingVel,
                                                })
                                        .times(m_inPerTick)),
                        new DualNum<>(
                                new double[] {
                                    headingDelta, headingVel,
                                }));

        m_lastParPos = parPosVel.position;
        m_lastPerpPos = perpPosVel.position;
        m_lastHeading = heading;

        return twist;
    }

    @Override
    public List<Encoder> getLeftEncoders() {
        return Collections.emptyList();
    }

    @Override
    public List<Encoder> getRightEncoders() {
        return Collections.emptyList();
    }

    @Override
    public List<Encoder> getParEncoders() {
        return Collections.singletonList(m_par);
    }

    @Override
    public List<Encoder> getPerpEncoders() {
        return Collections.singletonList(m_perp);
    }
}
