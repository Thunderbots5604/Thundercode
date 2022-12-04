package org.firstinspires.ftc.team5604.robotparts;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class AutoDriveTrain extends DriveTrain {

    //Target Location for where we're going to
    private double[] targetLocation;
    private double[] currentLocation;
    private double[] epsilons;

    //Scales
    private final double X_SCALE;
    private final double Y_SCALE;
    private final double ANGLE_SCALE;


    public AutoDriveTrain(HardwareMap map, String flMotor, String frMotor, String blMotor, String brMotor, double[] epsilons, double X_SCALE, double Y_SCALE, double ANGLE_SCALE) {
        super(map, flMotor, frMotor, blMotor, brMotor);
        targetLocation = new double[] {0, 0, 0};
        currentLocation = new double[] {0, 0, 0};
        this.epsilons = epsilons.clone();
        this.X_SCALE = X_SCALE;
        this.Y_SCALE = Y_SCALE;
        this.ANGLE_SCALE = ANGLE_SCALE;
        setZeroTicks();
    }

    public void setTargetLocation(double[] location) {
        targetLocation = location.clone();
    }

    public boolean moveToTargetLocation(double power) {
        if(positionWithin(currentLocation, targetLocation, epsilons)) {
            setPowersToZero();
            pushPowers();
            return true;
        }

        double[] targetDirections = rotateDirections(positionDifference(targetLocation, currentLocation));

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
        double[] relativeChange = new double[3];

        relativeChange[0] = X_SCALE * (tickChange[0] - tickChange[1] - tickChange[2] + tickChange[3]);
        relativeChange[1] = Y_SCALE * (tickChange[0] + tickChange[1] + tickChange[2] + tickChange[3]);
        relativeChange[2] = /*((ANGLE_SCALE * (tickChange[0] + tickChange[1] + tickChange[2] - tickChange[3])) % (2 * Math.PI)) - Math.PI*/ 0;

        double[] absoluteChange = rotateDirections(relativeChange);

        currentLocation[0] += absoluteChange[0];
        currentLocation[1] += absoluteChange[1];
        currentLocation[2] = addAngle(currentLocation[2], absoluteChange[2]);
    }

    public double[] rotateDirections(double[] directions) {
        double[] newDirections = new double[3];

        newDirections[0] = directions[0] * Math.cos(currentLocation[2]) - directions[1] * Math.sin(currentLocation[2]);
        newDirections[1] = directions[0] * Math.sin(currentLocation[2]) + directions[1] * Math.cos(currentLocation[2]);
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

    public double subtractAngle(double angle1, double angle2) {
        double difference = addAngle(angle1, -angle2);
        if (Math.abs(difference) > Math.PI / 2) {
            difference -= Math.signum(difference) * Math.PI / 2;
        }
        return difference;
    }

    //angles are in radians and is between -pi and +pi
    public double addAngle(double angle1, double angle2) {
        double angle = angle1 + angle2;
        if (Math.abs(angle) > Math.PI) {
            if (angle > Math.PI) {
                angle -= Math.PI * 2;
            } else if ((-1) * angle > Math.PI) {
                angle += Math.PI * 2;
            }
        }
        return angle;
    }

    public double[] getCurrentLocation() {
        return currentLocation.clone();
    }
}
