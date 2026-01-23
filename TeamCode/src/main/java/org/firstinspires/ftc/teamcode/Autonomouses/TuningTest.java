package org.firstinspires.ftc.teamcode.Autonomouses;

import org.firstinspires.ftc.teamcode.AutoEninge.MovementEngine;

public class TuningTest extends MovementEngine {
    @Override
    public void runPath() {
        drivePID(1, 0);
    }
}
