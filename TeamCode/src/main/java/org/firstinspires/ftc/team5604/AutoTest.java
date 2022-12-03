package org.firstinspires.ftc.team5604;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.team5604.robotparts.AutoDriveTrain;

@Autonomous(name="test")
public class AutoTest extends LinearOpMode {

    private AutoDriveTrain drive;

    @Override
    public void runOpMode() {
        drive = new AutoDriveTrain(hardwareMap, "fl", "fr", "bl", "br", new double[] {100, 100, 10}, 1, 1, 1);
        double[] target = new double[] {200, 200, 30};
        drive.setTargetLocation(target);
        while(!drive.moveToTargetLocation()) {
            drive.updateCurrentLocation();
        }
    }
}
