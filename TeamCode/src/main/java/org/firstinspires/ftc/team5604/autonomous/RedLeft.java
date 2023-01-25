package org.firstinspires.ftc.team5604.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team5604.autonomous.DetectSignalSleeve;
import org.firstinspires.ftc.team5604.robotparts.AutoDriveTrain;
import org.firstinspires.ftc.team5604.robotparts.Claw;
import org.firstinspires.ftc.team5604.robotparts.Armon2;
import org.firstinspires.ftc.team5604.Values;
import org.firstinspires.ftc.team5604.robotparts.AutoDriveTrainGyro;
import org.firstinspires.ftc.team5604.robotparts.Gyro;

@Autonomous(name = "RedLeft", group = "Autonomous")
public class RedLeft extends LinearOpMode {
    private DetectSignalSleeve camera;
    private int location;
    private Armon2 arm;
    private Claw claw;
    private AutoDriveTrainGyro drive;
    private ElapsedTime timer;
    private Gyro gyro;

    private final double ARM_STARTING_HEIGHT = 150;
    private final double ARM_MEDIUM_HEIGHT = 500;
    private final double ARM_POLE_HEIGHT = 1100;
    private int NUM_CYCLES = 2;
    private int counter = 0;


    private double[][] toStoragePositions = {{-16, 2, 0}, {-16, 2, Math.PI/2}, {-16, 36, Math.PI/2}, {-19, 36, Math.PI/2}};
    private double[][] toPolePositions = {{-17, 40, -Math.PI/4}, {1, 40, -Math.PI/4}};
    //[0]=onTheWayToStorage [1]=storage, [2]=pole


    enum DrivePosition {
        POSITION_START,
        POSITION_TOSTORAGE1,
        POSITION_TOSTORAGE2,
        POSITION_TOSTORAGE3,
        POSITION_TOSTORAGE4,
        POSITION_STORAGE,
        POSITION_TOPOLE1,
        POSITION_TOPOLE2,
        POSITION_POLE,
        POSITION_TOPARK1,
        POSITION_TOPARK2,
        POSITION_PARK
    }

    enum ArmState {
        ARM_START,
        ARM_TOSTORAGE,
        ARM_STORAGE,
        ARM_TOPOLE1,
        ARM_TOPOLE2,
        ARM_POLE,
        ARM_TOPARK,
        ARM_PARK
    }

    enum ClawState {
        CLAW_START,
        CLAW_TOSTORAGE,
        CLAW_STORAGE,
        CLAW_TOPOLE,
        CLAW_POLE,
        CLAW_TOPARK,
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
        drive = new AutoDriveTrainGyro(hardwareMap, "fl", "fr", "bl", "br", new double[] {Values.LATERAL_ERROR, Values.LONGITUDINAL_ERROR, 0.02}, Values.INCHES_PER_TICK_LATERAL, Values.INCHES_PER_TICK_LONGITUDINAL, Values.RADIANS_PER_TICK, gyro);
        claw = new Claw(hardwareMap, "claw", 0.85, 1);
        arm = new Armon2(hardwareMap, "am1", "am2", 10, 1100, 0.05);
        camera = new DetectSignalSleeve(hardwareMap, telemetry, this);
        camera.init();
        waitForStart();
        location = 2;//camera.findRegion();
        boolean timerStarted = false;

        sleep(500);
        timer = new ElapsedTime();

        while (opModeIsActive()) {
            telemetry.addData("robotState", drivePosition);
            telemetry.addData("armState", armState);
            telemetry.addData("clawState", clawState);

            switch (drivePosition) {
                case POSITION_START:
                    //if we don't want any cycles, we just directly go park
                    if(NUM_CYCLES == 0){
                        clawState = ClawState.CLAW_TOPARK;
                        armState = ArmState.ARM_TOPARK;
                        drivePosition = DrivePosition.POSITION_TOPARK1;
                    }
                    drive.setTargetLocation(new double[] {0, 2, 0}); //so robot doesn't scratch wall
                    if(!drive.moveToTargetLocation(.5)) {
                        drive.updateCurrentLocation();
                    } else {
                        drive.stop();
                        sleep(500);
                        drivePosition = DrivePosition.POSITION_TOSTORAGE1;
                    }
                    break;
                case POSITION_TOSTORAGE1:
                    drive.setTargetLocation(toStoragePositions[0]);
                    if(!drive.moveToTargetLocation(.5)) {
                        drive.updateCurrentLocation();
                    } else {
                        drivePosition = DrivePosition.POSITION_TOSTORAGE2;
                        sleep(100);
                    }
                    break;
                case POSITION_TOSTORAGE2:
                    drive.setTargetLocation(toStoragePositions[1]);
                    if(!drive.moveToTargetLocation(.5)) {
                        drive.updateCurrentLocation();
                    } else if(Math.abs(drive.getCurrentLocation()[2] - toStoragePositions[1][2]) < 0.02) {
                        drivePosition = DrivePosition.POSITION_TOSTORAGE3;
                    }
                    break;
                case POSITION_TOSTORAGE3:
                    drive.setTargetLocation(toStoragePositions[2]);
                    if(!drive.moveToTargetLocation(.5)) {
                        drive.updateCurrentLocation();
                    } else if(Math.abs(drive.getCurrentLocation()[2] - toStoragePositions[2][2]) < 0.02) {
                        drivePosition = DrivePosition.POSITION_TOSTORAGE4;
                    }
                    break;
                /*case POSITION_TOSTORAGE4:
                    drive.setTargetLocation(toStoragePositions[3]);
                    if(!drive.moveToTargetLocation(.2)) {
                        drive.updateCurrentLocation();
                    } else {
                        drivePosition = DrivePosition.POSITION_STORAGE;
                    }
                    break;
                case POSITION_STORAGE:
                    clawState = ClawState.CLAW_STORAGE;
                    break;
                case POSITION_TOPOLE1:
                    if(armState == ArmState.ARM_TOPOLE2 || armState == ArmState.ARM_POLE){
                        drive.setTargetLocation(toPolePositions[0]);
                        if(!drive.moveToTargetLocation(.5)) {
                            drive.updateCurrentLocation();
                        } else {
                            drivePosition = DrivePosition.POSITION_TOPOLE2;
                        }
                    }
                    break;
                case POSITION_TOPOLE2:
                    if(armState == ArmState.ARM_TOPOLE2 || armState == ArmState.ARM_POLE){
                        drive.setTargetLocation(toPolePositions[1]);
                        if(!drive.moveToTargetLocation(.5)) {
                            drive.updateCurrentLocation();
                        } else {
                            drivePosition = DrivePosition.POSITION_POLE;
                        }
                    }
                    break;
                case POSITION_POLE:
                    drive.stop();
                    break;
                case POSITION_TOPARK1:
                    drive.setTargetLocation(new double[] {0, 18, 0});
                    if(!drive.moveToTargetLocation(.5)){
                        drive.updateCurrentLocation();
                    }
                    break;
                case POSITION_TOPARK2:
                    if(location == 1) {
                        drive.setTargetLocation(new double[] {-18, 18, 0});
                    } else if(location == 3){
                        drive.setTargetLocation(new double[] {18, 18, 0});
                    } else {
                        drivePosition = DrivePosition.POSITION_PARK;
                    }
                    if(!drive.moveToTargetLocation(.5)){
                        drive.updateCurrentLocation();
                    } else {
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
            switch (armState) {
                case ARM_START:
                    armState = ArmState.ARM_TOSTORAGE;
                    break;
                case ARM_TOSTORAGE:
                    arm.setTargetPosition(ARM_STARTING_HEIGHT);
                    if(arm.moveToTarget()){
                        armState = ArmState.ARM_STORAGE;
                    }
                    break;
                case ARM_STORAGE:
                    arm.lock();
                    arm.moveToTarget();
                    break;
                case ARM_TOPOLE1:
                    arm.setTargetPosition(ARM_MEDIUM_HEIGHT);
                    if(arm.moveToTarget()){
                        armState = ArmState.ARM_TOPOLE2;
                    }
                    break;
                case ARM_TOPOLE2:
                    arm.setTargetPosition(ARM_POLE_HEIGHT);
                    if(arm.moveToTarget()){
                        armState = ArmState.ARM_POLE;
                    }
                    break;
                case ARM_POLE:
                    clawState = ClawState.CLAW_POLE;
                    arm.lock();
                    arm.moveToTarget();
                    break;
                case ARM_TOPARK:
                    arm.setTargetPosition(0);
                    if(arm.moveToTarget()){
                        armState = ArmState.ARM_PARK;
                    }
                    break;
                case ARM_PARK:
                    complete[1] = true;
                    break;
                default:
                    armState = ArmState.ARM_START;
                    break;
            }
            switch (clawState) {
                case CLAW_START:
                    claw.open();
                    clawState = ClawState.CLAW_TOSTORAGE;
                    break;
                case CLAW_TOSTORAGE:
                    //wait
                    break;
                case CLAW_STORAGE:
                    //closes claw only when both the arm is at the correct height and the robot is at
                    //the right location (since POSITION_STORAGE leads to CLAW_STORAGE, we can assume
                    //that the robot is already at the correct location; this if statement is only
                    //to check if the arm is at the storage)
                    if(armState == ArmState.ARM_STORAGE){
                        claw.close();
                        armState = ArmState.ARM_TOPOLE1;
                        clawState = ClawState.CLAW_TOPOLE;
                        drivePosition = DrivePosition.POSITION_TOPOLE1;
                    }
                    break;
                case CLAW_TOPOLE:
                    //wait
                    break;
                case CLAW_POLE:
                    //opens claw only when both the arm is at the correct height and the robot is at
                    //the pole (since ARM_POLE leads to CLAW_POLE, we can assume that the arm is already
                    //at the correct height; this if statement is only to check if the robot itself
                    //is at POSITION_POLE)
                    if(drivePosition == DrivePosition.POSITION_POLE) {
                        claw.open();
                        if(NUM_CYCLES - 1 > 0){
                            NUM_CYCLES--;
                            clawState = ClawState.CLAW_TOSTORAGE;
                            armState = ArmState.ARM_TOSTORAGE;
                            drivePosition = DrivePosition.POSITION_TOSTORAGE2;
                        } else {
                            clawState = ClawState.CLAW_TOPARK;
                            armState = ArmState.ARM_TOPARK;
                            drivePosition = DrivePosition.POSITION_TOPARK1;
                        }
                    }
                    break;
                case CLAW_TOPARK:
                    clawState = ClawState.CLAW_PARK;
                    break;
                case CLAW_PARK:
                    complete[2] = true;
                    break;
                default:
                    clawState = ClawState.CLAW_START;
                    break;*/
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
