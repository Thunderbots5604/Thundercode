package org.firstinspires.ftc.team5604.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.team5604.robotparts.AutoDriveTrain;
import org.firstinspires.ftc.team5604.autonomous.DetectSignalSleeve;
import org.firstinspires.ftc.team5604.Values;

@Autonomous(name = "Blue Cycles (State Machine)", group = "Autonomous")
public class BlueAuto extends LinearOpMode {
    private DetectSignalSleeve camera;
    private AutoDriveTrain drive;

    @Override
    public void runOpMode() {
        camera = new DetectSignalSleeve(hardwareMap, telemetry, this);
        camera.init();
        int parkingSpot, avgHue;
        drive = new AutoDriveTrain(hardwareMap, "fl", "fr", "bl", "br", new double[] {Values.LATERAL_ERROR, Values.LONGITUDINAL_ERROR, Values.ANGLE_ERROR}, Values.INCHES_PER_TICK_LATERAL, Values.INCHES_PER_TICK_LONGITUDINAL, Values.RADIANS_PER_TICK);
        double[] targetPosition = new double[3];
        double[] position;
        waitForStart();

        //sets parking location
        parkingSpot = camera.findRegion();
        drive.setTargetLocation(new double[] {0, 2, 0});
        while(!drive.moveToTargetLocation(.5)) {
            drive.updateCurrentLocation();
        }
        sleep(100);
        if(parkingSpot == 1) {
            targetPosition = new double[] {-16, 16, 0};
            drive.setTargetLocation(new double[] {-16, 2, 0});
            while(!drive.moveToTargetLocation(.5)) {
                drive.updateCurrentLocation();
            }
        } else if(parkingSpot == 2) {
            targetPosition = new double[] {0, 16, 0};
        } else if (parkingSpot == 3){
            targetPosition = new double[] {16, 16, 0};
            drive.setTargetLocation(new double[] {16, 2, 0});
            while(!drive.moveToTargetLocation(.5)) {
                drive.updateCurrentLocation();
            }
        }
        else {
            targetPosition = new double[] {0, 16, 0};
        }
        sleep(100);

        //parks in randomized location
        drive.setTargetLocation(targetPosition);
        while(!drive.moveToTargetLocation(.5)) {
            drive.updateCurrentLocation();
        }

        sleep(10000);

    }
}
