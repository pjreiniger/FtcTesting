package com.gosftc.lib.rr.messages;

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;

@SuppressWarnings({"PMD.DataClass", "PMD.FieldNamingConventions"})
public final class ThreeDeadWheelInputsMessage {
    public long timestamp;
    public PositionVelocityPair par0;
    public PositionVelocityPair par1;
    public PositionVelocityPair perp;

    public ThreeDeadWheelInputsMessage(
            PositionVelocityPair par0, PositionVelocityPair par1, PositionVelocityPair perp) {
        this.timestamp = System.nanoTime();
        this.par0 = par0;
        this.par1 = par1;
        this.perp = perp;
    }
}
