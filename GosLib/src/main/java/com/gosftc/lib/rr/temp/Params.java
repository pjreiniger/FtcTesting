package com.gosftc.lib.rr.temp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class Params {
        // IMU orientation
        // TODO: fill in these values based on
        //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        // drive model parameters
        public double inPerTick = 0;
        public double trackWidthTicks = 0;

        // feedforward parameters (in tick units)
        public double kS = 0;
        public double kV = 0;
        public double kA = 0;

        // path profile parameters (in inches)
        public double maxWheelVel = 50;
        public double minProfileAccel = -30;
        public double maxProfileAccel = 50;

        // turn profile parameters (in radians)
        public double maxAngVel = Math.PI; // shared with path
        public double maxAngAccel = Math.PI;

        // path controller gains
        public double ramseteZeta = 0.7; // in the range (0, 1)
        public double ramseteBBar = 2.0; // positive

        // turn controller gains
        public double turnGain = 0.0;
        public double turnVelGain = 0.0;
    }