package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class PoseStorage {
    public static Pose2d telePowerLeft = new Pose2d(0,0, Math.toRadians(270));
    public static Pose2d telePowerRight = new Pose2d(0, 0, Math.toRadians(270));


    public static Pose2d rightAuto = new Pose2d(37.5, -61.3, Math.toRadians(90));
    public static Pose2d leftAuto = new Pose2d(-37.5, -61.3, Math.toRadians(90));
}
