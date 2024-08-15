package com.gosftc.lib.rr.messages;

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import java.util.List;

@SuppressWarnings({"PMD.DataClass", "PMD.FieldNamingConventions"})
public final class TankLocalizerInputsMessage {
    public long timestamp;
    public PositionVelocityPair[] left;
    public PositionVelocityPair[] right;

    public TankLocalizerInputsMessage(
            List<PositionVelocityPair> left, List<PositionVelocityPair> right) {
        this.timestamp = System.nanoTime();
        this.left = left.toArray(new PositionVelocityPair[0]);
        this.right = right.toArray(new PositionVelocityPair[0]);
    }
}
