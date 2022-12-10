package org.firstinspires.ftc.team5604;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.team5604.robotparts.AutoDriveTrain;

@Autonomous(name="test")
public class AutoTest extends LinearOpMode {

    private AutoDriveTrain drive;

    @Override
    public void runOpMode() {
        drive = new AutoDriveTrain(hardwareMap, "fl", "fr", "bl", "br", new double[] {50, 50, .05}, .25, .25, -0.002);
        double[] target = new double[] {500, -1000, Math.PI / 3};
        drive.setTargetLocation(target);
        double[] position;
        waitForStart();
        while(!drive.moveToTargetLocation(.75)) {
            drive.updateCurrentLocation();
            position = drive.getCurrentLocation();
            telemetry.addData("x", position[0]);
            telemetry.addData("y", position[1]);
            telemetry.addData("Angle", position[2]);
            telemetry.update();
        }

        sleep(10000);
    }
}
