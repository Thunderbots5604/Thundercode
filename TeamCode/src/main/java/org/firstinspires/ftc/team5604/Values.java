package org.firstinspires.ftc.team5604;

public class Values {
    /*
    Values for scaling how much the robot moves
    RADIANS_PER_TICK is what multiplies the left side - right side encoder ticks (ie rotational motion)
    INCHES_PER_TICK_LONGITUDINAL is what multiplies the sum of all encoder ticks (ie forwards and backwards motion)
    INCHES_PER_TICK_LATERAL is what multiplies the (fl + br) - (fr + bl) encoder ticks (ie side to side motion)
     */
    public static final double RADIANS_PER_TICK = -(2 * Math.PI / 360) * 0.029;
    public static final double INCHES_PER_TICK_LONGITUDINAL = 28.0 / 5129;
    public static final double INCHES_PER_TICK_LATERAL = ((33.75 / 6875) + (32.0 / 6527)) / 2;
    /*
    Values for error on where the robot completes a given moveToTargetLocation
    Higher values means the robot will be less precise in getting to target positions
    Lower values means the robot will be more likely to become extremely jittery, which may also lead to inaccuracy
    Probably set these as low as possible without destroying the robot's movement completely
    Also note that ANGLE_ERROR is measured in radians
    ANGLE_ERROR should be pretty small, but LONGITUDINAL_ERROR and LATERAL_ERROR should probably be greater than 10*INCHES_PER_TICK, probably
     */
    public static final double ANGLE_ERROR = 25 * ((4 * Math.PI / 360) * 0.029);
    public static final double LONGITUDINAL_ERROR = 75 * (28.0 / 5129);
    public static final double LATERAL_ERROR = 75 * (((33.75 / 6875) + (32.0 / 6527)) / 2);
    /*
    Values for Armon2 target positions
    GROUND is literally having a cone on the ground, so it should probably be 0
    GROUND_JUNCTION might be a little bit higher, since the ground junctions are a little off the ground
    LOW_JUNCTION, MEDIUM_JUNCTION, and HIGH_JUNCTION are the positions needed to drop the cone off at those junctions
    ABOVE_CAMERA is used when a cone is preloaded to move it out of the way of the cam so we can detect which place to park with a preload
    FOUR_STACK is used to pick up cones at the top of the 4 stack
     */
     
    public static final double TILE_LENGTH = 18;


     
    public static final double GROUND = 0;
    public static final double GROUND_JUNCTION = 209;
    public static final double LOW_JUNCTION = 507.5;
    public static final double MEDIUM_JUNCTION = 742;
    public static final double HIGH_JUNCTION = 1150;
    public static final double ABOVE_CAMERA = 600;
    public static final double FOUR_STACK = 275;


    private Values() {}
}
