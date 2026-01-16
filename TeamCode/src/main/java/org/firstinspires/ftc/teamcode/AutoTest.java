package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoEninge.AutoEngine;

@Autonomous(name="TuneTest")
public class AutoTest extends AutoEngine {

    @Override
    public void runPath() {
        strafeMove(1,0);
    }
}
