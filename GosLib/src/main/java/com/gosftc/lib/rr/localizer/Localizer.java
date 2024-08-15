package com.gosftc.lib.rr.localizer;

import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import java.util.List;

public interface Localizer {
    Twist2dDual<Time> update();

    List<Encoder> getLeftEncoders();

    List<Encoder> getRightEncoders();

    List<Encoder> getParEncoders();

    List<Encoder> getPerpEncoders();
}
