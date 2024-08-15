package com.gosftc.lib.rr.drive;

import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.gosftc.lib.rr.localizer.MecanumDriveLocalizer;
import com.gosftc.lib.rr.messages.DriveCommandMessage;
import com.gosftc.lib.rr.messages.MecanumCommandMessage;
import com.gosftc.lib.rr.messages.PoseMessage;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.Arrays;
import java.util.List;

public class MecanumDrive {
    private final DcMotorEx m_flMotor;
    private final DcMotorEx m_frMotor;
    private final DcMotorEx m_blMotor;
    private final DcMotorEx m_brMotor;

    private final Encoder m_flEncoder;
    private final Encoder m_frEncoder;
    private final Encoder m_blEncoder;
    private final Encoder m_brEncoder;

    private final DownsampledWriter m_targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter m_driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter m_mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);


    public MecanumDrive(HardwareMap hardwareMap) {
        m_flMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        m_frMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
        m_blMotor = hardwareMap.get(DcMotorEx.class, "leftBack");
        m_brMotor = hardwareMap.get(DcMotorEx.class, "rightBack");

        m_flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        m_flEncoder = new OverflowEncoder(new RawEncoder(m_flMotor));
        m_frEncoder = new OverflowEncoder(new RawEncoder(m_frMotor));
        m_blEncoder = new OverflowEncoder(new RawEncoder(m_blMotor));
        m_brEncoder = new OverflowEncoder(new RawEncoder(m_brMotor));
    }

    public void stop() {
        setDrivePower(0, 0, 0, 0);
    }

    public void setDrivePower(double fl, double fr, double bl, double br) {
        // Scale the results so no motor is saturated
        double maxPowerMag = 1;
        maxPowerMag = Math.max(fl, maxPowerMag);
        maxPowerMag = Math.max(fr, maxPowerMag);
        maxPowerMag = Math.max(bl, maxPowerMag);
        maxPowerMag = Math.max(br, maxPowerMag);

        m_flMotor.setPower(fl / maxPowerMag);
        m_frMotor.setPower(fr / maxPowerMag);
        m_blMotor.setPower(bl / maxPowerMag);
        m_brMotor.setPower(br / maxPowerMag);
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        setDrivePower(
                wheelVels.leftFront.get(0),
                wheelVels.rightFront.get(0),
                wheelVels.leftBack.get(0),
                wheelVels.rightBack.get(0)
        );
    }

    public void publishDriveCommand(DriveCommandMessage driveCommandMessage) {
        m_driveCommandWriter.write(driveCommandMessage);
    }

    public void publishMecanumCommand(MecanumCommandMessage mecanumCommandMessage) {
        m_mecanumCommandWriter.write(mecanumCommandMessage);
    }

    public void publishTargetPose(PoseMessage poseMessage) {
        m_targetPoseWriter.write(poseMessage);
    }

    public MecanumDriveLocalizer createOdometryLocalizer(MecanumKinematics m_kinematics, IMU imu) {
        return new MecanumDriveLocalizer(m_kinematics, m_flEncoder, m_frEncoder, m_blEncoder, m_brEncoder, imu);
    }

    public List<DcMotorEx> getLeftMotors() {
        return Arrays.asList(
                m_flMotor,
                m_blMotor
        );
    }

    public List<DcMotorEx> getRightMotors() {
        return Arrays.asList(
                m_frMotor,
                m_brMotor
        );
    }
}
