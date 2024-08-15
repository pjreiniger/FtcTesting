package com.gosftc.lib.rr.localizer;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;

public class Pose2dState {
    public final Pose2d m_pose;
    public final PoseVelocity2d m_velocity;

    public Pose2dState(Pose2d pose, PoseVelocity2d vel) {
        m_pose = pose;
        m_velocity = vel;
    }
}
