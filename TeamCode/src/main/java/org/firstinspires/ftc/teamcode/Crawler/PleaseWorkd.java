package org.firstinspires.ftc.teamcode.Crawler;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Crawler.core.CrawlerRobot;

public class PleaseWorkd {

    CrawlerRobot robot = new CrawlerRobot.Builder(null)
            .frontLeft("fl")
            .frontRight("fr")
            .backLeft("bl")
            .backRight("br")
            .motors()
            .withThreeDeadWheels("enc_l", "enc_r", "enc_c")
            .trackWidth(13.0)
            .centerWheelOffset(3.5)
            .build();
}
