package org.firstinspires.ftc.team5604.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team5604.autonomous.DetectSignalSleeve;
import org.firstinspires.ftc.team5604.robotparts.Claw;
import org.firstinspires.ftc.team5604.robotparts.Armon2;
import org.firstinspires.ftc.team5604.Values;
import org.firstinspires.ftc.team5604.robotparts.AutoDriveTrainGyro;
import org.firstinspires.ftc.team5604.robotparts.Gyro;

@Autonomous(name = "Pre-Load-Medium-Left", group = "Autonomous")
public class PreLoadMedLeft extends LinearOpMode {
    private DetectSignalSleeve camera;
    private int location;
    private Armon2 arm;
    private Claw claw;
    private AutoDriveTrainGyro drive;
    private ElapsedTime timer;
    private Gyro gyro;
    private boolean armPole3Toggle = false;

    private double[][] positions = {{17, 2, 0}, {17, 18, 0}, {9, 19, 0}};
    private double[] parkPositions = new double[3];

    enum DrivePosition {
        POSITION_START,
        POSITION_TO_POLE1,
        POSITION_TO_POLE2,
        POSITION_TO_POLE3,
        POSITION_POLE,
        POSITION_TO_PARK,
        POSITION_PARK
    }

    enum ArmState {
        ARM_START,
        ARM_TO_POLE,
        ARM_POLE1,
        ARM_POLE2,
        ARM_POLE3,
        ARM_TO_PARK,
        ARM_PARK
    }

    enum ClawState {
        CLAW_START,
        CLAW_TO_POLE,
        CLAW_POLE,
        CLAW_TO_PARK,
        CLAW_PARK
    }

    volatile DrivePosition drivePosition = DrivePosition.POSITION_START;
    volatile ArmState armState = ArmState.ARM_START;
    volatile ClawState clawState = ClawState.CLAW_START;

    private boolean[] complete = {false, false, false};

    @Override
    public void runOpMode() throws InterruptedException {
        //init stuff here
        gyro = new Gyro(hardwareMap);
        drive = new AutoDriveTrainGyro(hardwareMap, "fl", "fr", "bl", "br", new double[] {Values.LATERAL_ERROR, Values.LONGITUDINAL_ERROR, 0.03}, Values.INCHES_PER_TICK_LATERAL, Values.INCHES_PER_TICK_LONGITUDINAL, Values.RADIANS_PER_TICK, gyro);
        claw = new Claw(hardwareMap, "claw", 0.85, 1);
        arm = new Armon2(hardwareMap, "am1", "am2", 10, 1100, 0.05);
        camera = new DetectSignalSleeve(hardwareMap, telemetry, this);
        camera.init();
        waitForStart();

        claw.close();
        sleep(100);
        /*drive.setTargetLocation(new double[] {0, 2, 0});
        while(!drive.moveToTargetLocation(.5)){
            drive.updateCurrentLocation();
        }*/
        arm.setTargetPosition(Values.ABOVE_CAMERA);
        while(arm.getCurrentPosition() < Values.ABOVE_CAMERA - 10) {
            arm.moveToTarget();
        }
        location = camera.findRegion();
        arm.move(1);
        if(location == 1){
            parkPositions = new double[] {-18, 18, 0};
        } else if(location == 3){
            parkPositions = new double[] {18, 18, 0};
        } else {
            parkPositions = new double[] {0, 18, 0};
        }
        boolean timerStarted = false;

        sleep(500);
        timer = new ElapsedTime();

        while (opModeIsActive()) {
            telemetry.addData("robotState", drivePosition);
            telemetry.addData("armState", armState);
            telemetry.addData("clawState", clawState);
            telemetry.addData("location", location);

            switch (drivePosition) {
                case POSITION_START:
                    drivePosition = DrivePosition.POSITION_TO_POLE1;
                    break;
                case POSITION_TO_POLE1:
                    drive.setTargetLocation(positions[0]);
                    if(!drive.moveToTargetLocation(.5)) {
                        drive.updateCurrentLocation();
                    } else {
                        sleep(1000);
                        drivePosition = DrivePosition.POSITION_TO_POLE2;
                    }
                    break;
                case POSITION_TO_POLE2:
                    drive.setTargetLocation(positions[1]);
                    if(!drive.moveToTargetLocation(.5)) {
                        drive.updateCurrentLocation();
                    } else {
                        sleep(1000);
                        drivePosition = DrivePosition.POSITION_TO_POLE3;
                    }
                    break;
                case POSITION_TO_POLE3:
                    drive.setTargetLocation(positions[2]);
                    if(!drive.moveToTargetLocation(.5)) {
                        drive.updateCurrentLocation();
                    } else {
                        sleep(1000);
                        drivePosition = DrivePosition.POSITION_POLE;
                        armState = ArmState.ARM_POLE1;
                    }
                    break;
                case POSITION_POLE:
                    break;
                case POSITION_TO_PARK:
                    drive.setTargetLocation(parkPositions);
                    if(!drive.moveToTargetLocation(.5)) {
                        drive.updateCurrentLocation();
                    } else {
                        sleep(1000);
                        drivePosition = DrivePosition.POSITION_PARK;
                    }
                    break;
                case POSITION_PARK:
                    complete[0] = true;
                    break;
                default:
                    drivePosition = DrivePosition.POSITION_START;
                    break;
            }

            switch(armState){
                case ARM_START:
                    armState = ArmState.ARM_TO_POLE;
                    break;
                case ARM_TO_POLE:
                    break;
                case ARM_POLE1:
                    arm.setTargetPosition(900);
                    if(arm.moveToTarget()){
                        armState = ArmState.ARM_POLE2;
                    }
                    break;
                case ARM_POLE2:
                    arm.setTargetPosition(800);
                    if(arm.moveToTarget()){
                        armState = ArmState.ARM_POLE3;
                        armPole3Toggle = false;
                    }
                    break;
                case ARM_POLE3:
                    /*if(!armPole3Toggle) {
                        arm.lock();
                        armPole3Toggle = true;
                    }*/
                    arm.lock();
                    arm.moveToTarget();
                    break;
                case ARM_TO_PARK:
                    arm.setTargetPosition(0);
                    if(arm.moveToTarget()){
                        armState = ArmState.ARM_PARK;
                    }
                    break;
                case ARM_PARK:
                    arm.move(0);
                    complete[1] = true;
                    break;
                default:
                    armState = ArmState.ARM_START;
                    break;
            }

            switch(clawState){
                case CLAW_START:
                    clawState = ClawState.CLAW_TO_POLE;
                    break;
                case CLAW_TO_POLE:
                    clawState = ClawState.CLAW_POLE;
                    break;
                case CLAW_POLE:
                    if(armState == ArmState.ARM_POLE3 && drivePosition == DrivePosition.POSITION_POLE){
                        sleep(1000);
                        claw.open();
                        sleep(1000);
                        clawState = ClawState.CLAW_TO_PARK;
                        armState = ArmState.ARM_TO_PARK;
                        drivePosition = DrivePosition.POSITION_TO_PARK;
                    }
                    break;
                case CLAW_TO_PARK:
                    clawState = ClawState.CLAW_PARK;
                    break;
                case CLAW_PARK:
                    complete[2] = true;
                    break;
                default:
                    clawState = ClawState.CLAW_START;
                    break;
            }

            double[] position;
            drive.updateCurrentLocation();
            position = drive.getCurrentLocation();
            telemetry.addData("x", position[0]);
            telemetry.addData("y", position[1]);
            telemetry.addData("Angle", position[2]);
            telemetry.update();
            if (complete[0] && complete[1] && complete[2]) {
                stop();
            }
        }
    }
}
