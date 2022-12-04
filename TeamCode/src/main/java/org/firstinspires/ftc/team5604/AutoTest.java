package org.firstinspires.ftc.team5604;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.team5604.robotparts.AutoDriveTrain;

@Autonomous(name="test")
public class AutoTest extends LinearOpMode {

    private AutoDriveTrain drive;

    @Override
    public void runOpMode() {
        drive = new AutoDriveTrain(hardwareMap, "fl", "fr", "bl", "br", new double[] {100, 100, 1}, 1, 1, 0.01);
        double[] target = new double[] {0, 0, 3};
        drive.setTargetLocation(target);
        double[] position;
        waitForStart();
        while(!drive.moveToTargetLocation(.35)) {
            drive.updateCurrentLocation();
            position = drive.getCurrentLocation();
            telemetry.addData("x", position[0]);
            telemetry.addData("y", position[1]);
            telemetry.addData("Angle", position[2]);
            telemetry.update();
        }
    }
}
