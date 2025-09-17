package org.firstinspires.ftc.teamcode.drive.opmode;

// RR-specific imports

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.concurrent.atomic.AtomicReference;


@Autonomous(name = "AutonJalen2")
public class AutonJalen2 extends LinearOpMode {
    private static final int TOWER_MAX_POSITION = 1900;
    private static final int TOWER_MIN_POSITION = 0;

    private static final int ARM_MIN_POSITION = 40;
    private static final int ARM_MAX_POSITION = 320;

    private static final double BUCKET_TAKE_POSITION = 0;
    private static final double BUCKET_DROP_POSITION = 1;

    private static final double CLAW_CLOSED_POSITION = 0.3;
    private static final double CLAW_OPEN_POSITION = 0.8;

    private static final double WRIST_INTAKE_POSITION = 0.7;
    private static final double WRIST_DROP_POSITION = 0.1;
    private DcMotor tower = null;
    private DcMotor arm = null;
    private Servo bucket = null;
    private Servo claw = null;
    private Servo wrist = null;
    public int targetTower = 0;
    public int targetArm = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the drive system (uses Road Runner's SampleMecanumDrive)
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // tower init
        tower = hardwareMap.get(DcMotor.class, "tower");
        tower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tower.setDirection(DcMotor.Direction.REVERSE);
        tower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tower.setTargetPosition(targetTower);
        tower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tower.setPower(1);
        // arm init
        arm = hardwareMap.get(DcMotor.class, "arm");
        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setTargetPosition(targetArm);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1);
        // servos init
        bucket = hardwareMap.get(Servo.class, "bucket");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(CLAW_OPEN_POSITION);

        // Define the poses of the robot
        Pose2d startPose = new Pose2d(-30, -60, Math.toRadians(90));
        Pose2d netZoneDropPose1 = new Pose2d(-56, -54, Math.toRadians(45)); // in front of net zone for basket scoring
        Pose2d netZoneDropPose2 = new Pose2d(-55, -54, Math.toRadians(45));
        Pose2d netZoneDropPose3 = new Pose2d(-55, -54, Math.toRadians(45));
        Pose2d netZoneDropPose4 = new Pose2d(-54, -54, Math.toRadians(45));
        AtomicReference<Pose2d> currentDropPose = new AtomicReference<>(netZoneDropPose1);
        Pose2d firstSpike = new Pose2d(-44, -46, Math.toRadians(90));
        Pose2d secondSpike = new Pose2d(-51, -45, Math.toRadians(90));
        Pose2d thirdSpike = new Pose2d(-56, -44, Math.toRadians(100));
        Pose2d parkPose = new Pose2d(-22, 0, Math.toRadians(0));
        // Set the robot's starting position in the drive system
        drive.setPoseEstimate(startPose);

        // Create the trajectory sequence
        TrajectorySequence preloadToBasket = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    telemetry.addData("STEP:", "to net zone");
                    telemetry.update();
                })
                .lineToLinearHeading(netZoneDropPose1)
                .addTemporalMarker(0.1, () -> { // Execute tower movement halfway through the trajectory
                    tower.setTargetPosition(TOWER_MAX_POSITION);
                    tower.setPower(1);
                    telemetry.addData("Tower", "Moving to max position");
                    telemetry.update();
                })
                .build();

        TrajectorySequence toFirstSpike = drive.trajectorySequenceBuilder(netZoneDropPose1)
                // go from net zone to the first spike
                .lineToLinearHeading(firstSpike)
                .addTemporalMarker(0.5,()->{ // time for the arm to reach its target and then open the claw to drop the sample
                    claw.setPosition(CLAW_OPEN_POSITION);
                    telemetry.addData("STEP:", "to first spike");
                    telemetry.update();
                })
                .build();
        TrajectorySequence firstSpikeToBasket = drive.trajectorySequenceBuilder(firstSpike)
                .addTemporalMarker(0.5,()->{ // time for the arm to reach its target and then open the claw to drop the sample
                    claw.setPosition(CLAW_OPEN_POSITION);
                    currentDropPose.set(netZoneDropPose2);
                    telemetry.addData("STEP:", "to net zone");
                    telemetry.update();
                })
                // go from first spike to the basket
                .lineToLinearHeading(netZoneDropPose2)
                .waitSeconds(0.5)
                .build();
        TrajectorySequence toSecondSpike = drive.trajectorySequenceBuilder(netZoneDropPose2)
                .addTemporalMarker(()->{ // time for the arm to reach its target and then open the claw to drop the sample
                    telemetry.addData("STEP:", "to second spike");
                    telemetry.update();
                })
                .lineToLinearHeading(secondSpike)
                .build();
        TrajectorySequence secondSpikeToBasket = drive.trajectorySequenceBuilder(secondSpike)
                .addTemporalMarker(0.5,()->{ // time for the arm to reach its target and then open the claw to drop the sample
                    claw.setPosition(CLAW_OPEN_POSITION);
                    currentDropPose.set(netZoneDropPose3);
                    telemetry.addData("STEP:", "to net zone");
                    telemetry.update();
                })
                // go from first spike to the basket
                .lineToLinearHeading(netZoneDropPose3)

                .waitSeconds(0.5)
                .build();
        TrajectorySequence toThirdSpike = drive.trajectorySequenceBuilder(netZoneDropPose3)
                .lineToLinearHeading(thirdSpike)
                .addTemporalMarker(()->{ // time for the arm to reach its target and then open the claw to drop the sample
                    telemetry.addData("STEP:", "to third spike");
                    telemetry.update();
                })
                .build();
        TrajectorySequence thirdSpikeToBasket = drive.trajectorySequenceBuilder(thirdSpike)
                .addTemporalMarker(0.5,()->{ // time for the arm to reach its target and then open the claw to drop the sample
                    claw.setPosition(CLAW_OPEN_POSITION);
                    currentDropPose.set(netZoneDropPose4);
                    telemetry.addData("STEP:", "to net zone");
                    telemetry.update();
                })
                // go from first spike to the basket
                .lineToLinearHeading(netZoneDropPose4)
                .waitSeconds(0.5)
                .build();
        TrajectorySequence parkBySubmersible = drive.trajectorySequenceBuilder(netZoneDropPose4)
                .lineToLinearHeading(parkPose) // park by submersible
                .waitSeconds(3)
                .build();
        ////////////////// MULTIPLE-USE TRAJECTORY SEQUENCES/////////////////////////
        // for proper timing of bucket movement
        TrajectorySequence bucketDrop = drive.trajectorySequenceBuilder(currentDropPose.get())
                .addTemporalMarker(1, ()->{
                    bucket.setPosition(BUCKET_DROP_POSITION);
                })
                .waitSeconds(1.5)
                .build();

        TrajectorySequence pickUpTime = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0.9, ()->{
                    claw.setPosition(CLAW_CLOSED_POSITION);
                })
                .waitSeconds(1)
                .build();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            ///////// SCORE PRELOAD /////////////
            drive.followTrajectorySequence(preloadToBasket); // bot moves to net zone, and raises tower


            drive.followTrajectorySequence(bucketDrop); // timing for bucket to drop sample
            towerRetract();
            ////////////////// FIRST SPIKE /////////////////////
            drive.followTrajectorySequence(toFirstSpike); // moves to first spike
            pickUpSpike(); // move arm down
            drive.followTrajectorySequence(pickUpTime); // time for claw to pick up the sample
            dropOffSpike(); // move arm up to bucket
            drive.followTrajectorySequence(firstSpikeToBasket); // drop sample in bucket and move toward the net zone
            towerDeposit(); // tower goes up
            drive.followTrajectorySequence(bucketDrop); // bucket flips to score sample
            towerRetract(); // tower goes down
            ////////////////// SECOND SPIKE ////////////////////
            drive.followTrajectorySequence(toSecondSpike);
            pickUpSpike();
            drive.followTrajectorySequence(pickUpTime);
            dropOffSpike();
            drive.followTrajectorySequence(secondSpikeToBasket);
            towerDeposit();
            drive.followTrajectorySequence(bucketDrop);
            towerRetract();
            /////////////////// THIRD SPIKE ///////////////////
            drive.followTrajectorySequence(toThirdSpike);
            pickUpSpike();
            drive.followTrajectorySequence(pickUpTime);
            dropOffSpike();
            drive.followTrajectorySequence(thirdSpikeToBasket);
            towerDeposit();
            drive.followTrajectorySequence(bucketDrop);
            towerRetract();
            drive.followTrajectorySequence(parkBySubmersible);
            //////////////////////// END /////////////////////////
        }
    }
    void towerDeposit() {
        tower.setTargetPosition(TOWER_MAX_POSITION);
    }
    void towerRetract() {
        telemetry.addData("Tower Position", tower.getCurrentPosition());
        claw.setPosition(CLAW_OPEN_POSITION);
        tower.setTargetPosition(TOWER_MIN_POSITION);
        bucket.setPosition(BUCKET_TAKE_POSITION);
    }
    void pickUpSpike() {
        arm.setTargetPosition(ARM_MAX_POSITION);
        claw.setPosition(CLAW_OPEN_POSITION);
        wrist.setPosition(WRIST_INTAKE_POSITION);
    }
    void dropOffSpike() {
        arm.setTargetPosition(ARM_MIN_POSITION);
        wrist.setPosition(WRIST_DROP_POSITION);
    }
}
