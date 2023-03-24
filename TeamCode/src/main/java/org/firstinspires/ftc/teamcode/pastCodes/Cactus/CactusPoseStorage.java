package org.firstinspires.ftc.teamcode.pastCodes.Cactus;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class CactusPoseStorage {
    public static Pose2d telePowerLeft = new Pose2d(0,0, Math.toRadians(270));
    public static Pose2d telePowerRight = new Pose2d(0, 0, Math.toRadians(90));


    public static Pose2d rightAutoRed = new Pose2d(40, -60, Math.toRadians(90));
    public static Pose2d leftAutoRed = new Pose2d(-36.25, -62.5, Math.toRadians(90));

    public static Pose2d rightAutoBlue = new Pose2d(-36.25, 62.5, Math.toRadians(90));
    public static Pose2d leftAutoBlue = new Pose2d(40, 60, Math.toRadians(90));
}