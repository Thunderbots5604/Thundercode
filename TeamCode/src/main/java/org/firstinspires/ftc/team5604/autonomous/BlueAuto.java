package org.firstinspires.ftc.team5604.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.team5604.robotparts.DriveTrain;

@Autonomous(name = "Blue Cycles (State Machine)", group = "Autonomous")
public class BlueAuto extends LinearOpMode {
    private DetectSignalSleeve camera;
    private int region;
    DriveTrain drive;

    @Override
    public void runOpMode() {
        camera = new DetectSignalSleeve(hardwareMap, telemetry, this);
        int parkingSpot;
        drive = new DriveTrain(hardwareMap, "fl", "fr", "bl", "br");
        waitForStart();

        parkingSpot = camera.findRegion();


    }
}
