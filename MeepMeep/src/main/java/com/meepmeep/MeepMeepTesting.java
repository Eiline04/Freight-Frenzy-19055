package com.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class MeepMeepTesting {

    static Pose2d startPose = new Pose2d(-40.085, -63.54, radians(270.0));
    static Pose2d shippingHubPose = new Pose2d(-9.0, -48.0, radians(265.0));

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, Math.toRadians(600), Math.toRadians(600), 10.0)
                .setDimensions(12.59, 16.14).build();

        TrajectorySequence duck = duck(myBot);
        TrajectorySequence preload = levelThreePreload(duck.getEnd(), myBot);

        //myBot.followTrajectorySequence(duck);
        //myBot.followTrajectorySequence(preload);
        myBot.followTrajectorySequence(cycles(preload.getEnd(), myBot));

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    static TrajectorySequence cycles(Pose2d initalPose, RoadRunnerBotEntity d) {
        return d.getDrive().trajectorySequenceBuilder(initalPose)
                //START OF CYCLE
                .splineToSplineHeading(new Pose2d(6.0, -68.0, radians(-5.0)), radians(-5.0))
                .addDisplacementMarker(() -> {
                })
                .setVelConstraint(new TranslationalVelocityConstraint(20.0))
                .splineToSplineHeading(new Pose2d(50.0, -69.0, radians(10.0)), radians(10.0))
                .waitSeconds(1.0)

                //go back now
                .resetVelConstraint()
                .setReversed(true)
                .setVelConstraint(new TranslationalVelocityConstraint(20.0))
                .splineToSplineHeading(new Pose2d(15.84, -71.0, radians(0.0)), radians(180.0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                })
                .resetVelConstraint()
                .splineToSplineHeading(shippingHubPose, radians(95.0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                })
                .waitSeconds(0.8)
                .addDisplacementMarker(() -> {

                })
                .setReversed(false)
                .resetVelConstraint()
                .build();

    }

    static TrajectorySequence duck(RoadRunnerBotEntity d) {
        return d.getDrive().trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                })
                .lineToLinearHeading(new Pose2d(-55.23, -59.0, radians(270.0)))
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                })
                .build();
    }

    static TrajectorySequence levelThreePreload(Pose2d initialPose, RoadRunnerBotEntity d) {
        return d.getDrive().trajectorySequenceBuilder(initialPose)
                .setVelConstraint(new TranslationalVelocityConstraint(60))
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                })
                .lineToLinearHeading(shippingHubPose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                })
                .waitSeconds(0.7)
                .addDisplacementMarker(() -> {
                })
                .resetVelConstraint()
                .build();
    }

    static double radians(double deg) {
        return Math.toRadians(deg);
    }
}