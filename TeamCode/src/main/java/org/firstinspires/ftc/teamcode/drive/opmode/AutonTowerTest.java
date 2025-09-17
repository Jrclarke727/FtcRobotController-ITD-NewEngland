package org.firstinspires.ftc.teamcode.drive.opmode;



// RR-specific imports

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name = "AutonTowerTest")
public class AutonTowerTest extends LinearOpMode {
    private static final int TOWER_MAX_POSITION = 4900;
    private static final int TOWER_MIN_POSITION = 0;

    private static final int ARM_MIN_POSITION = -300;
    private static final int ARM_MAX_POSITION = -20;

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
    private int bucketPos = 0;
    public int targetTower = 0;
    public int targetArm = -250;
    private boolean towerUp = false;


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the drive system (uses Road Runner's SampleMecanumDrive)
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // tower init
        tower = hardwareMap.get(DcMotor.class, "tower");
        tower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

        // Define the starting pose of the robot
        Pose2d startPose = new Pose2d(-34, -60, Math.toRadians(90));
        Pose2d dropOffPose = new Pose2d(-34, -59, Math.toRadians(90));

        // Set the robot's starting position in the drive system
        drive.setPoseEstimate(startPose);

        // Create the trajectory sequence
        TrajectorySequence dropOffPreload = drive.trajectorySequenceBuilder(startPose)
                // Move to high junction and simulate scoring
                // starting position: robot's right side aligned with seam between second and third tile
                .lineToLinearHeading(dropOffPose)
                .addTemporalMarker(0.5,()->{
                    claw.setPosition(CLAW_OPEN_POSITION);
        })
                .waitSeconds(1)
                // robot moves in front of net zone, put sample already loaded into basket
                // Creates a path to set coordinates and sets heading to 45deg

                .build();
        ////////////////// MULTIPLE-USE TRAJECTORY SEQUENCES/////////////////////////
        // for proper timing of bucket movement
        TrajectorySequence bucketDrop = drive.trajectorySequenceBuilder(dropOffPose)
                .addTemporalMarker(2, ()->{
                    bucket.setPosition(BUCKET_DROP_POSITION);
                })
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(-34, -58, Math.toRadians(90)))
                .waitSeconds(0.5)
                .build();

        TrajectorySequence pickUpTime = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0.5, ()->{
                    claw.setPosition(CLAW_CLOSED_POSITION);
                })
                .waitSeconds(1)
                .build();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            // Follow the trajectory sequence
            pickUpSpike(); // arm down, claw open, wrist down
            drive.followTrajectorySequence(pickUpTime); // claw close after one sec
            dropOffSpike(); // arm up, wrist up
            drive.followTrajectorySequence(dropOffPreload); // open claw to drop sample
            towerDeposit(); // tower up
            drive.followTrajectorySequence(bucketDrop); // bucket drop
            towerRetract(); // tower down
        }
        while (opModeIsActive()) {
//            if (-4800 > tower.getCurrentPosition()){ // once tower reaches the upper position, flip bucket up
//                bucketPos = 1;
//                bucket.setPosition(bucketPos);
//                towerUp = true;
//            }
//            else {
//                bucketPos = 0;
//                bucket.setPosition(bucketPos);
//                towerUp = false;
//            }
            telemetry.addLine("Tower telemetry:");
            telemetry.addData("Tower Power", tower.getPower());
            telemetry.addData("Tower Position", tower.getCurrentPosition());
            telemetry.addData("TARGET TOWER", targetTower);
            telemetry.addData("tower up", towerUp);
            telemetry.addData("bucket position", bucketPos);
            telemetry.update();
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

