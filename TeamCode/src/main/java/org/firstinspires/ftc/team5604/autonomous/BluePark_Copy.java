package org.firstinspires.ftc.team5604.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.team5604.robotparts.AutoDriveTrain;
import org.firstinspires.ftc.team5604.robotparts.Armon2;
import org.firstinspires.ftc.team5604.robotparts.Claw;
import org.firstinspires.ftc.team5604.autonomous.DetectSignalSleeve;

@Autonomous(name = "Blue Park test", group = "Autonomous")
public class BluePark_Copy extends LinearOpMode {
    private DetectSignalSleeve camera;
    private int region;
    private AutoDriveTrain drive;
    private Claw claw;
    private Armon2 arm;

    @Override
    public void runOpMode() {
        region = 0;
        camera = new DetectSignalSleeve(hardwareMap, telemetry, this);
        camera.init();
        
        claw = new Claw(hardwareMap, "claw", 0.85, 1);
        arm = new Armon2(hardwareMap, "am1", "am2", 10, 1100, 0.05);
        
        drive = new AutoDriveTrain(hardwareMap, "fl", "fr", "bl", "br", new double[] {50, 50, 10}, 1, 1, 0.01);
        double[] targetPosition = new double[3];
        double[] position;
        telemetry.update();
        waitForStart();
        claw.close();
        sleep(500);
        sleep(1000);
        arm.setTargetPosition(500);
        
        while (region == 0) {
            region = camera.findRegion();
            arm.moveToTarget();
        }
        drive.setTargetLocation(new double[] {0, 200, 0});
        while(!drive.moveToTargetLocation(.5)) {
            drive.updateCurrentLocation();
            arm.moveToTarget();
        }
        sleep(100);
        if(region == 1) {
            targetPosition = new double[] {-3100, 2700, 0};
            //for parking spots 1 and 3, can't move straight diagonally because of obstacles
            //so we move left/right first then up
            drive.setTargetLocation(new double[] {-3100, 200, 0});
            while(!drive.moveToTargetLocation(.5)) {
                drive.updateCurrentLocation();
                position = drive.getCurrentLocation();
                telemetry.addData("Position", region);
                telemetry.addData("x", position[0]);
                telemetry.addData("y", position[1]);
                telemetry.addData("Angle", position[2]);
                telemetry.update();
                arm.moveToTarget();
            }
        } else if(region == 2) {
            targetPosition = new double[] {0, 2700, 0};
        } else if (region == 3){
            targetPosition = new double[] {3100, 2700, 0};
            
            drive.setTargetLocation(new double[] {3100, 200, 0});
            while(!drive.moveToTargetLocation(.5)) {
                drive.updateCurrentLocation();
                position = drive.getCurrentLocation();
                telemetry.addData("Position", region);
                telemetry.addData("x", position[0]);
                telemetry.addData("y", position[1]);
                telemetry.addData("Angle", position[2]);
                telemetry.update();
                arm.moveToTarget();
            }
        }
        sleep(100);

        //parks in randomized location
        drive.setTargetLocation(targetPosition);
        while(!drive.moveToTargetLocation(.5)) {
            drive.updateCurrentLocation();
            position = drive.getCurrentLocation();
            telemetry.addData("Position", region);
            telemetry.addData("x", position[0]);
            telemetry.addData("y", position[1]);
            telemetry.addData("Angle", position[2]);
            telemetry.update();
            arm.moveToTarget();
        }
        
        sleep(10000);

    }
}
