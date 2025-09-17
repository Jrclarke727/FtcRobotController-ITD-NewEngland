package com.example.meepmeepdeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepDeep {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(75, 80, Math.toRadians(180), Math.toRadians(180), 17)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-30, -60, Math.toRadians(90)))
                        // starting position: robot's right side aligned with seam between second and third tile
                        .lineToLinearHeading(new Pose2d(-54, -53, Math.toRadians(45)))
                        // robot moves in front of net zone, put sample already loaded into basket
                        .waitSeconds(1.5) //this time will be for the tower to drop in the sample
                        .setTurnConstraint(20,20) // set turn constraint only really works for .turn speed
                        .lineToLinearHeading(new Pose2d(-46, -44, Math.toRadians(90)))
                        .waitSeconds(1)
                        // pick up first spike (yellow sample)
                        .lineToLinearHeading(new Pose2d(-54, -53, Math.toRadians(45)))
                        /* TO DO:
                        - get tower working in autonomous
                        - get arm working in autonomous
                        - get servos working */
                        .waitSeconds(1.5)
                        .lineToLinearHeading(new Pose2d(-59, -41, Math.toRadians(90)))

                        .waitSeconds(1)

                        .lineToLinearHeading(new Pose2d(-54, -53, Math.toRadians(45)))

                        .waitSeconds(1.5)
                        // in front of yellow sample next to the wall
                        .lineToLinearHeading(new Pose2d(-58, -41, Math.toRadians(100)))

                        .waitSeconds(1)

                        .lineToLinearHeading(new Pose2d(-54, -53, Math.toRadians(45)))

                        .waitSeconds(1.5)

                        .lineToLinearHeading(new Pose2d(28,-60,0))

                        .waitSeconds(1)

                        .lineToLinearHeading(new Pose2d(-54, -54, Math.toRadians(45)))

                        .waitSeconds(1)

                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}