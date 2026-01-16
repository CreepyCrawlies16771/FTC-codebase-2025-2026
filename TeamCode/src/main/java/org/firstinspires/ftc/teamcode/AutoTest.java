package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoEninge.AutoEngine;

@Autonomous(name="TuneTest")
public class AutoTest extends AutoEngine {

    @Override
    public void runPath() {
        strafePID(3, 0);
        strafePID(-1, 0);
        drivePID(-1, 0);
        drivePID(1, 0);
    }
}
