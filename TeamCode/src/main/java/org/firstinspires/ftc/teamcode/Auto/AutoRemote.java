package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Detection.CameraThread;
import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDriveImpl;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.wrappers.DuckMechanism;
import org.firstinspires.ftc.teamcode.wrappers.Intake;
import org.firstinspires.ftc.teamcode.wrappers.Lifter;
import org.firstinspires.ftc.teamcode.wrappers.TapeTurret;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Autonomous
public class AutoRemote extends LinearOpMode {
    MecanumDriveImpl drive;
    Intake intake;
    Lifter lifter;
    TapeTurret turret;
    DuckMechanism duckMechanism;

    OpenCvCamera webcam;
    CameraThread cameraThread;
    Lifter.LEVEL result;

    Pose2d startPose = new Pose2d(-40.085, -63.54, radians(270.0));
    Pose2d shippingHubPose = new Pose2d(-9.0 - 2.0, -48.0 + 1.1, radians(265.0));

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDriveImpl(hardwareMap);
        lifter = new Lifter(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry, null);
        turret = new TapeTurret(hardwareMap);
        duckMechanism = new DuckMechanism(hardwareMap);

        Thread updater = new Thread(new Updater());

        initWebcam();
        sleep(1000);
        cameraThread = new CameraThread(webcam);
        Thread cameraRunner = new Thread(cameraThread);
        cameraRunner.start();

        cameraThread.setState(CameraThread.CAMERA_STATE.INIT);
        sleep(1000);
        cameraThread.setState(CameraThread.CAMERA_STATE.STREAM);

        telemetry.addLine("Ready!");
        telemetry.update();

        TrajectorySequence duck = duck();
        TrajectorySequence three = levelThreePreload(duck.end());
        TrajectorySequence two = levelTwoPreload(duck.end());
        TrajectorySequence one = levelOnePreload(duck.end());

        waitForStart();
        //detect go brr
        result = CameraThread.getResult();
        telemetry.addData("Result", result);
        telemetry.update();

        drive.setPoseEstimate(startPose);
        updater.start(); //start calling update for intake and lifter

        drive.followTrajectorySequence(duck);
        intake.raiseIntake();

        if (result == Lifter.LEVEL.FIRST) {
            drive.followTrajectorySequence(one);

            lifter.closeBox();
            lifter.goToPosition(700, Lifter.LEVEL.DOWN.ticks);
            duckMechanism.stopSpin();
            intake.raiseIntake(800);
            drive.followTrajectorySequence(cycles(one.end()));
        } else if (result == Lifter.LEVEL.SECOND) {
            drive.followTrajectorySequence(two);

            lifter.closeBox();
            lifter.goToPosition(300, Lifter.LEVEL.DOWN.ticks);
            duckMechanism.stopSpin();

            drive.followTrajectorySequence(cycles(two.end()));
        } else {
            drive.followTrajectorySequence(three);

            lifter.closeBox();
            lifter.goToPosition(0, Lifter.LEVEL.DOWN.ticks);
            duckMechanism.stopSpin();

            drive.followTrajectorySequence(cycles(three.end()));
        }

        PoseStorage.currentPose = drive.getPoseEstimate();
    }

    TrajectorySequence cycles(Pose2d initalPose) {
        return drive.trajectorySequenceBuilder(initalPose)
                //START OF CYCLE 1
                .splineToSplineHeading(new Pose2d(6.0, -69.0, radians(0.0)), radians(0.0))
                .addDisplacementMarker(() -> {
                    intake.lowerIntake();
                    intake.startIntake();
                })
                .setVelConstraint(new TranslationalVelocityConstraint(30.0))
                .splineToSplineHeading(new Pose2d(50.0 - 5.0, -72.0, radians(10.0)), radians(10.0))
                .waitSeconds(0.3)

                //go back now
                .resetVelConstraint()
                .setReversed(true)
                .setVelConstraint(new TranslationalVelocityConstraint(35.0))
                .splineToSplineHeading(new Pose2d(15.84, -71.0, radians(0.0)), radians(180.0))
                .addDisplacementMarker(() -> {
                    intake.raiseIntake();
                    intake.stopIntake();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> lifter.goToPosition(0, Lifter.LEVEL.THIRD.ticks))

                .splineToSplineHeading(shippingHubPose, radians(95.0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lifter.dumpingBox.setPosition(0.8))
                .waitSeconds(0.7)
                .addDisplacementMarker(() -> {
                    lifter.closeBox();
                    lifter.goToPosition(0, Lifter.LEVEL.DOWN.ticks);
                })
                .setReversed(false)
                .resetVelConstraint()

                //START OF CYCLE 2
                .splineToSplineHeading(new Pose2d(6.0, -69.0, radians(0.0)), radians(0.0))
                .addDisplacementMarker(() -> {
                    intake.lowerIntake();
                    intake.startIntake();
                })
                .setVelConstraint(new TranslationalVelocityConstraint(30.0))
                .splineToSplineHeading(new Pose2d(50.0 - 3.0, -72.0, radians(20.0)), radians(20.0))
                .waitSeconds(0.3)

                //go back now
                .resetVelConstraint()
                .setReversed(true)
                .setVelConstraint(new TranslationalVelocityConstraint(30.0))
                .splineToSplineHeading(new Pose2d(15.84, -71.0, radians(0.0)), radians(180.0))
                .addDisplacementMarker(() -> {
                    intake.raiseIntake();
                    intake.stopIntake();

                })
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> lifter.goToPosition(0, Lifter.LEVEL.THIRD.ticks))

                .splineToSplineHeading(shippingHubPose, radians(95.0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lifter.dumpingBox.setPosition(0.8))
                .waitSeconds(0.7)
                .addDisplacementMarker(() -> {
                    lifter.closeBox();
                    lifter.goToPosition(0, Lifter.LEVEL.DOWN.ticks);
                })
                .setReversed(false)
                .resetVelConstraint()

                //START OF CYCLE 3

                .splineToSplineHeading(new Pose2d(6.0, -69.0, radians(0.0)), radians(0.0))
                .addDisplacementMarker(() -> {
                    intake.lowerIntake();
                    intake.startIntake();
                })
                .setVelConstraint(new TranslationalVelocityConstraint(30.0))
                .splineToSplineHeading(new Pose2d(50.0, -72.0, radians(30.0)), radians(30.0))
                .waitSeconds(0.3)

                //go back now
                .resetVelConstraint()
                .setReversed(true)
                .setVelConstraint(new TranslationalVelocityConstraint(35.0))
                .splineToSplineHeading(new Pose2d(15.84, -71.0, radians(0.0)), radians(180.0))
                .addDisplacementMarker(() -> {
                    intake.raiseIntake();
                    intake.stopIntake();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> lifter.goToPosition(0, Lifter.LEVEL.THIRD.ticks))

                .splineToSplineHeading(shippingHubPose, radians(95.0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lifter.dumpingBox.setPosition(0.8))
                .waitSeconds(0.7)
                .addDisplacementMarker(() -> {
                    lifter.closeBox();
                    lifter.goToPosition(0, Lifter.LEVEL.DOWN.ticks);
                })
                .setReversed(false)
                .resetVelConstraint()

                //PARK
                .splineToSplineHeading(new Pose2d(6.0, -69.0, radians(0.0)), radians(0.0))
                .addDisplacementMarker(() -> {
                    intake.raiseIntake();
                    intake.stopIntake();
                })
                .splineToSplineHeading(new Pose2d(40.0, -72.0, radians(10.0)), radians(10.0))
                .build();

    }

    TrajectorySequence duck() {
        return drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> duckMechanism.startSpin())
                .lineToLinearHeading(new Pose2d(-55.23 + 1.5 - 1, -59.0, radians(270.0)))
                .waitSeconds(1)
                .build();
    }

    TrajectorySequence levelThreePreload(Pose2d initialPose) {
        return drive.trajectorySequenceBuilder(initialPose)
                .setVelConstraint(new TranslationalVelocityConstraint(60))
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> lifter.goToPosition(0, Lifter.LEVEL.THIRD.ticks))
                .lineToLinearHeading(shippingHubPose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lifter.dumpingBox.setPosition(0.8))
                .waitSeconds(0.5)
                .resetVelConstraint()
                .build();
    }

    TrajectorySequence levelTwoPreload(Pose2d initialPose) {
        return drive.trajectorySequenceBuilder(initialPose)
                .setVelConstraint(new TranslationalVelocityConstraint(60))
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> lifter.goToPosition(0, Lifter.LEVEL.SECOND.ticks))
                .lineToLinearHeading(shippingHubPose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lifter.dumpingBox.setPosition(0.8))
                .waitSeconds(0.5)
                .resetVelConstraint()
                .build();
    }

    TrajectorySequence levelOnePreload(Pose2d initialPose) {
        return drive.trajectorySequenceBuilder(initialPose)
                .setVelConstraint(new TranslationalVelocityConstraint(60))
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    lifter.goToPosition(0, Lifter.LEVEL.FIRST.ticks);
                    intake.lowerIntake();
                })
                .lineToLinearHeading(new Pose2d(-9.0 - 2.0, -45.0, radians(265.0)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lifter.dumpingBox.setPosition(0.8))
                .waitSeconds(0.5)
                .resetVelConstraint()
                .build();
    }

    static double radians(double deg) {
        return Math.toRadians(deg);
    }

    public void initWebcam() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
    }

    class Updater implements Runnable {
        @Override
        public void run() {
            while (opModeIsActive()) {
                lifter.update();
                intake.update();
                telemetry.update();
            }
        }
    }

}