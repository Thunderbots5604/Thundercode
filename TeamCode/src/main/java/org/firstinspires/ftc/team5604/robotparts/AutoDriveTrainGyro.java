package org.firstinspires.ftc.team5604.robotparts;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class AutoDriveTrainGyro extends DriveTrain {

    //Target Location for where we're going to
    private double[] targetLocation;
    private double[] currentLocation;
    private double[] epsilons;

    //Scales
    private final double X_SCALE;
    private final double Y_SCALE;
    private final double ANGLE_SCALE;

    private Gyro gyro;
    private double zeroAngle;


    public AutoDriveTrainGyro(HardwareMap map, String flMotor, String frMotor, String blMotor, String brMotor, double[] epsilons, double X_SCALE, double Y_SCALE, double ANGLE_SCALE, Gyro gyro) {
        super(map, flMotor, frMotor, blMotor, brMotor);
        targetLocation = new double[] {0, 0, 0};
        currentLocation = new double[] {0, 0, 0};
        this.epsilons = epsilons.clone();
        this.X_SCALE = X_SCALE;
        this.Y_SCALE = Y_SCALE;
        this.ANGLE_SCALE = ANGLE_SCALE;
        setZeroTicks();
        this.gyro = gyro;
        zeroAngle = gyro.getHeading();
    }

    public void setTargetLocation(double[] location) {
        targetLocation = location.clone();
    }

    public boolean moveToTargetLocation(double power) {
        double[] targetDirections = new double[3];

        if(Math.abs(currentLocation[0] - targetLocation[0]) < epsilons[0] && Math.abs(currentLocation[1] - targetLocation[1]) < epsilons[1]) {
            if(Math.abs(addAngle(currentLocation[2], -targetLocation[2])) < epsilons[2]) {
                setPowersToZero();
                pushPowers();
                return true;
            }
            else {
                targetDirections[0] = 0;
                targetDirections[1] = 0;
                targetDirections[2] = addAngle(targetLocation[2], -currentLocation[2]);

                targetDirections[2] *= -1 / Math.abs(targetDirections[2]);
                calculatePower(targetDirections);
                normalizePowers();
                scalePowers(power);
                pushPowers();

                return false;
            }
        }

        targetDirections = rotateDirections(positionDifference(targetLocation, currentLocation), -currentLocation[2]);
        double slowFactor = Math.sqrt(targetDirections[0] * targetDirections[0] + targetDirections[1] * targetDirections[1]);
        double scale = (1 / Math.PI) * slowFactor;
        targetDirections[0] /= scale;
        targetDirections[1] /= scale;
        if(slowFactor < X_SCALE * 200 + Y_SCALE * 200) {
            targetDirections[0] *= slowFactor / (X_SCALE * 200 + Y_SCALE * 200);
            targetDirections[1] *= slowFactor / (X_SCALE * 200 + Y_SCALE * 200);
        }

        targetDirections[2] *= -1;
        calculatePower(targetDirections);
        normalizePowers();
        scalePowers(power);
        pushPowers();

        return false;
    }

    public void updateCurrentLocation() {
        int[] tickChange = getTickChange();
        setZeroTicks();
        double[] relativeChange = new double[2];

        relativeChange[0] = X_SCALE * (tickChange[0] - tickChange[1] - tickChange[2] + tickChange[3]);
        relativeChange[1] = Y_SCALE * (tickChange[0] + tickChange[1] + tickChange[2] + tickChange[3]);

        double[] absoluteChange = rotateDirections(new double[] {relativeChange[0], relativeChange[1], currentLocation[2]}, currentLocation[2]);

        currentLocation[0] += absoluteChange[0];
        currentLocation[1] += absoluteChange[1];
        currentLocation[2] = addAngle(gyro.getHeading(), -zeroAngle);
    }

    public double[] rotateDirections(double[] directions, double angle) {
        double[] newDirections = new double[3];

        newDirections[0] = directions[0] * Math.cos(angle) - directions[1] * Math.sin(angle);
        newDirections[1] = directions[0] * Math.sin(angle) + directions[1] * Math.cos(angle);
        newDirections[2] = directions[2];

        return newDirections;
    }

    public boolean positionWithin(double[] position1, double[] position2, double[] range) {
        double[] difference = positionDifference(position1, position2);

        difference[0] = Math.abs(difference[0]);
        difference[1] = Math.abs(difference[1]);
        difference[2] = Math.abs(difference[2]);

        if(difference[0] <= range[0] && difference[1] <= range[1] && difference[2] <= range[2]) {
            return true;
        }
        else {
            return false;
        }
    }

    public double[] positionDifference(double[] current, double[] target) {
        double[] difference = new double[3];

        difference[0] = current[0] - target[0];
        difference[1] = current[1] - target[1];
        difference[2] = addAngle(current[2], -target[2]);

        return difference;
    }

    //angles are in radians and is between -pi and +pi
    public double addAngle(double angle1, double angle2) {
        double sum = angle1 + angle2;
        if(sum > Math.PI) {
            sum -= Math.PI;
        }
        if(sum < -Math.PI) {
            sum += Math.PI;
        }

        return sum;
    }

    public double[] getCurrentLocation() {
        return currentLocation.clone();
    }
}
