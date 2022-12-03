package org.firstinspires.ftc.team5604.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.team5604.robotparts.AutoDriveTrain;
import org.firstinspires.ftc.team5604.autonomous.DetectSignalSleeve;

@Autonomous(name = "Blue Cycles (State Machine)", group = "Autonomous")
public class BlueAuto extends LinearOpMode {
    private DetectSignalSleeve camera;
    private int region;
    private AutoDriveTrain drive;

    @Override
    public void runOpMode() {
        camera = new DetectSignalSleeve(hardwareMap, telemetry, this);
        camera.init();
        int parkingSpot, avgHue;
        drive = new AutoDriveTrain(hardwareMap, "fl", "fr", "bl", "br", new double[] {50, 50, 10}, 1, 1, 0.01);
        double[] targetPosition = new double[3];
        double[] position;
        waitForStart();

        //sets parking location
        parkingSpot = 3/*camera.findRegion()*/;
        drive.setTargetLocation(new double[] {0, 200, 0});
        while(!drive.moveToTargetLocation(.5)) {
            drive.updateCurrentLocation();
        }
        sleep(100);
        if(parkingSpot == 1) {
            targetPosition = new double[] {-3100, 2700, 0};
            //for parking spots 1 and 3, can't move straight diagonally because of obstacles
            //so we move left/right first then up
            drive.setTargetLocation(new double[] {-3100, 200, 0});
            while(!drive.moveToTargetLocation(.5)) {
                drive.updateCurrentLocation();
                position = drive.getCurrentLocation();
                telemetry.addData("Position", parkingSpot);
                telemetry.addData("x", position[0]);
                telemetry.addData("y", position[1]);
                telemetry.addData("Angle", position[2]);
                telemetry.update();
            }
        } else if(parkingSpot == 2) {
            targetPosition = new double[] {0, 2700, 0};
        } else if (parkingSpot == 3){
            targetPosition = new double[] {3100, 2700, 0};

            drive.setTargetLocation(new double[] {3100, 200, 0});
            while(!drive.moveToTargetLocation(.5)) {
                drive.updateCurrentLocation();
                position = drive.getCurrentLocation();
                telemetry.addData("Position", parkingSpot);
                telemetry.addData("x", position[0]);
                telemetry.addData("y", position[1]);
                telemetry.addData("Angle", position[2]);
                telemetry.update();
            }
        }
        sleep(100);

        //parks in randomized location
        drive.setTargetLocation(targetPosition);
        while(!drive.moveToTargetLocation(.5)) {
            drive.updateCurrentLocation();
            position = drive.getCurrentLocation();
            telemetry.addData("Position", parkingSpot);
            telemetry.addData("x", position[0]);
            telemetry.addData("y", position[1]);
            telemetry.addData("Angle", position[2]);
            telemetry.update();
        }

        sleep(10000);

    }
}
