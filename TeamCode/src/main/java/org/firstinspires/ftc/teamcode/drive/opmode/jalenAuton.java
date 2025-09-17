package org.firstinspires.ftc.teamcode.drive.opmode;



// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.geometry.Vector2d;


// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name = "jalenAuton", group = "Competition")
public class jalenAuton extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the drive system (uses Road Runner's SampleMecanumDrive)
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Define the starting pose of the robot
        Pose2d startPose = new Pose2d(-34, -60, Math.toRadians(90));

        // Set the robot's starting position in the drive system
        drive.setPoseEstimate(startPose);

        // Create the trajectory sequence
        TrajectorySequence trajectorySequence = drive.trajectorySequenceBuilder(startPose)
                // Move to high junction and simulate scoring
                .lineToLinearHeading(new Pose2d(-56, -55, Math.toRadians(45)))
                .waitSeconds(3)

                // First cycle: Move to neutral spot and return
                .lineToLinearHeading(new Pose2d(-48, -41, Math.toRadians(90)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-56, -55, Math.toRadians(45)))
                .waitSeconds(3)

                // Second cycle: Similar neutral spot to return
                .lineToLinearHeading(new Pose2d(-58, -41, Math.toRadians(90)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-56, -55, Math.toRadians(45)))
                .waitSeconds(3)

                // Final cycle: Move to parking
                .lineToLinearHeading(new Pose2d(-59, -35, Math.toRadians(135)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-56, -55, Math.toRadians(45)))
                .waitSeconds(3)

                // Parking
                .lineToLinearHeading(new Pose2d(-26, 0, Math.toRadians(0)))
                .waitSeconds(3)

                .build();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            // Follow the trajectory sequence
            drive.followTrajectorySequence(trajectorySequence);
        }
    }
}
