package com.gosftc.lib.rr.drive;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.TankKinematics;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.gosftc.lib.rr.messages.DriveCommandMessage;
import com.gosftc.lib.rr.messages.PoseMessage;
import com.gosftc.lib.rr.messages.TankCommandMessage;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.Arrays;
import java.util.List;

public class TankDrive {
    private final DownsampledWriter targetPoseWriter =
            new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter =
            new DownsampledWriter("DRIVE_COMMAND", 50_000_000);

    private final DownsampledWriter tankCommandWriter =
            new DownsampledWriter("TANK_COMMAND", 50_000_000);

    private final List<DcMotorEx> m_leftMotors;
    private final List<DcMotorEx> m_rightMotors;

    public TankDrive(HardwareMap hardwareMap) {
        m_leftMotors = Arrays.asList(hardwareMap.get(DcMotorEx.class, "left"));
        m_rightMotors = Arrays.asList(hardwareMap.get(DcMotorEx.class, "right"));

        for (DcMotorEx m : m_leftMotors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        for (DcMotorEx m : m_rightMotors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void stop() {
        setMotorPower(0, 0);
    }

    public void setMotorPower(double leftPower, double rightPower) {
        for (DcMotorEx m : m_leftMotors) {
            m.setPower(leftPower);
        }
        for (DcMotorEx m : m_rightMotors) {
            m.setPower(rightPower);
        }
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        TankKinematics.WheelVelocities<Time> wheelVels =
                new TankKinematics(2).inverse(PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        for (DcMotorEx m : m_leftMotors) {
            m.setPower(wheelVels.left.get(0) / maxPowerMag);
        }
        for (DcMotorEx m : m_rightMotors) {
            m.setPower(wheelVels.right.get(0) / maxPowerMag);
        }
    }

    public void publishTargetPose(PoseMessage message) {
        driveCommandWriter.write(message);
    }

    public void publishDriveCommand(DriveCommandMessage message) {
        tankCommandWriter.write(message);
    }

    public void publishTankCommand(TankCommandMessage message) {
        targetPoseWriter.write(message);
    }

    public List<DcMotorEx> getLeftMotors() {
        return m_leftMotors;
    }

    public List<DcMotorEx> getRightMotors() {
        return m_rightMotors;
    }
}
