package org.firstinspires.ftc.teamcode.Autonomouses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoEninge.RobotOrient.MovementEngine;

@Autonomous(name="BlueSecondAuto")
public class blueTriangleAuto extends MovementEngine {

    @Override
    public void runPath() throws InterruptedException {

        // Generated Path (robot-oriented)
        // Start: x=1.67 y=-0.01 h=0.0deg

        turnPID(0); // face start heading
        drivePID(2.18, 178);
        turnPID(50); // waypoint heading
        robot.shootSequence();


    }
}
