package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="MechCatTeleOp", group="Linear Opmode")
public class MechCatTeleOp extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor tower = null;
    private DcMotor arm = null;
    private Servo wrist = null;
    private Servo claw = null;
    private Servo bucket = null;

    // Arm and Wrist target positions for each state
    // private static final int ARM_POSITION_INIT = 300;
    // private static final int ARM_POSITION_WALL_GRAB = 1100;
    // private static final int ARM_POSITION_WALL_UNHOOK = 1700;
    // private static final int ARM_POSITION_HOVER_HIGH = 2600;
    // private static final int ARM_POSITION_CLIP_HIGH = 2100;
    // private static final int ARM_POSITION_LOW_BASKET = 2500;


    private static final int WRIST_POSITION_INIT = 0;
    private static final int WRIST_POSITION_SAMPLE = 270;
    private static final int WRIST_POSITION_SPEC = 10;

    private static final int TOWER_MAX_POSITION = 4900;
    private static final int TOWER_MIN_POSITION = 0;

    private static final int ARM_MIN_POSITION = -300;
    private static final int ARM_MAX_POSITION = 0;

    private static final double BUCKET_TAKE_POSITION = 0;
    private static final double BUCKET_DROP_POSITION = 1;

    private static final double CLAW_CLOSED_POSITION = 0.3;
    private static final double CLAW_OPEN_POSITION = 0.8;

    private static final double WRIST_INTAKE_POSITION = 0.7;
    private static final double WRIST_DROP_POSITION = 0.1;

    // Enum for state machine
    private enum RobotState {
        INIT,
        INTAKE,
        WALL_GRAB,
        WALL_UNHOOK,
        HOVER_HIGH,
        CLIP_HIGH,
        LOW_BASKET,
        MANUAL
    }

    // Initial state
    private RobotState currentState = RobotState.INIT;

    // Claw toggle state
    private boolean bucketTake = false;
    private boolean clawOpen = false;
    private boolean wristDown = false;
    private boolean lastBumpRight = false;
    private boolean lastBumpLeft = false;
    private boolean lastGrab = false;
    private boolean lastIntake = false;

    // Target position
    private int targetArm = -300;
    private int targetTower = 0;

    // Motor Power
    private double frontLeftPower = 0;
    private double frontRightPower = 0;
    private double backLeftPower = 0;
    private double backRightPower = 0;
    private double towerPower = 1;
    private double armPower = 0.5;

    // PID Constants
    private double Kp = 0.004;
    private double Ki = 0.02;
    private double Kd = 0.006;

    // Other PID things
    private double integralSum = 0;
    private double lastError = 0;
    private double encoderPosition = 0;
    private double error = 0;
    private double derivative = 0;
    private boolean setPointIsNotReached = false;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables.
        frontLeftMotor  = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor  = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        tower = hardwareMap.get(DcMotor.class, "tower");
        arm = hardwareMap.get(DcMotor.class, "arm");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");
        bucket = hardwareMap.get(Servo.class, "bucket");

        // Stop and reset encoders
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //tower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motors to use encoders
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tower.setTargetPosition(targetTower);
        tower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setTargetPosition(targetArm);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set motor direction
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        //Set zero power behavior
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // variables
        boolean towerExtended = false;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // State machine logic
            // switch (currentState) {
            //     case INIT:
            //         targetArm = ARM_POSITION_INIT;
            //         targetWrist = WRIST_POSITION_INIT;
            //         telemetry.addData("State", "INIT");
            //         break;
            //     case INTAKE:
            //         targetArm = ARM_POSITION_INTAKE;
            //         targetWrist = WRIST_POSITION_SAMPLE;
            //         telemetry.addData("State", "INTAKE");
            //         break;

            //     case WALL_GRAB:
            //         targetArm = ARM_POSITION_WALL_GRAB;
            //         targetWrist = WRIST_POSITION_SPEC;
            //         telemetry.addData("State", "WALL_GRAB");
            //         break;

            //     case WALL_UNHOOK:
            //         targetArm = ARM_POSITION_WALL_UNHOOK;
            //         targetWrist = WRIST_POSITION_SPEC;
            //         telemetry.addData("State", "WALL_UNHOOK");
            //         break;

            //     case HOVER_HIGH:
            //         targetArm = ARM_POSITION_HOVER_HIGH;
            //         targetWrist = WRIST_POSITION_SPEC;
            //         telemetry.addData("State", "HOVER_HIGH");
            //         break;

            //     case CLIP_HIGH:
            //         targetArm = ARM_POSITION_CLIP_HIGH;
            //         targetWrist = WRIST_POSITION_SPEC;
            //         telemetry.addData("State", "CLIP_HIGH");
            //         break;
            //     case LOW_BASKET:
            //         targetArm = ARM_POSITION_LOW_BASKET;
            //         targetWrist = WRIST_POSITION_SAMPLE;
            //         telemetry.addData("State", "LOW_BASKET");
            //         break;
            //     case MANUAL:
            //         telemetry.addData("State", "MANUAL");
            //         break;
            // }




            // Handle state transitions based on gamepad input
            // if (gamepad1.a) {
            //     if (targetTower > 0){
            //     targetTower -= 1.5;
            //     }
            // } else if (gamepad1.b) {
            //     if (targetTower < 2500) {
            //     targetTower += 1.5;
            //     }
            //}
            if (tower.getCurrentPosition() >= 50 && !towerExtended){
                towerExtended = true;
            }
            else if (tower.getCurrentPosition() >= 4850 && !towerExtended){
                towerExtended = false;
            }

            // Tower Controls
            if (gamepad1.x && !towerExtended) {
                targetTower = TOWER_MAX_POSITION;
            }
            else if (gamepad1.x && towerExtended) {
                targetTower = TOWER_MIN_POSITION;
            }
            else if (gamepad1.y) {
                targetTower = tower.getCurrentPosition();
            }
            //If tower is moving over max or under min height, stop moving
            // else if ((tower.getCurrentPosition() <= TOWER_MAX_POSITION && towerExtended) || (tower.getCurrentPosition() >= TOWER_MIN_POSITION && !towerExtended)) {
            //     towerPower = 0;
            // }
            // // Do not nest if statements beyond this point ^

            // //Arm Controls
            if (gamepad1.dpad_down){
                targetArm = -300;
            }
            else if (gamepad1.dpad_up){
                targetArm = -20;
                wrist.setPosition(WRIST_INTAKE_POSITION);
            }

            // //If arm is moving over max or under min height, stop moving
            // else if (arm.getCurrentPosition() >= 0 || arm.getCurrentPosition() <= -60){
            //     armPower = 0;
            // }
            // //If arm is moving over max or under min height, stop moving
            // else if (arm.getCurrentPosition() >= -10){
            //     armPower = 0.2;
            // }
            // else if (arm.getCurrentPosition() <= -50){
            //     armPower = -0.2;
            // }
            // else if (arm.getCurrentPosition() <= -100){
            //     armPower = 0.07;
            // }
            // else {
            //     armPower = 0;
            // }
            // Toggle bucket position when right_bumper is pressed
            if (gamepad1.left_bumper && !lastBumpLeft) {
                bucketTake= !bucketTake;
                if (bucketTake) {
                    bucket.setPosition(BUCKET_TAKE_POSITION);
                } else {
                    bucket.setPosition(BUCKET_DROP_POSITION);
                }
            }
            lastBumpLeft = gamepad1.left_bumper;


            lastGrab = gamepad1.b;
            boolean lastHook = gamepad1.y;

            // Toggle claw position when right_bumper is pressed
            if (gamepad1.right_bumper && !lastBumpRight) {
                clawOpen = !clawOpen;
                if (clawOpen) {
                    claw.setPosition(CLAW_OPEN_POSITION);
                } else {
                    claw.setPosition(CLAW_CLOSED_POSITION);
                }
            }
            lastBumpRight = gamepad1.right_bumper;

            if (gamepad1.b && !lastIntake) {
                wristDown = !wristDown;
                if (wristDown) {
                    wrist.setPosition(WRIST_DROP_POSITION);
                } else {
                    wrist.setPosition(WRIST_INTAKE_POSITION);
                }
            }
            lastIntake = gamepad1.b;

            // Mecanum drive mode
            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            // Set drivetrain power
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            // Send Power to Motors
            frontLeftMotor.setPower(frontLeftPower * -0.85);
            frontRightMotor.setPower(frontRightPower * -0.85);
            backLeftMotor.setPower(backLeftPower * -0.85);
            backRightMotor.setPower(backRightPower * -0.85);

            // Arm Control Loop (PID)

            // if (gamepad1.dpad_down && targetArm < ARM_MAX_POSITION){
            //     targetArm += 1;
            // }
            // else if (gamepad1.dpad_down && targetArm > ARM_MIN_POSITION){
            //     targetArm -= 1;
            // }

            // ElapsedTime timer = new ElapsedTime();
            // if (targetArm == arm.getCurrentPosition()) {
            //     setPointIsNotReached = false;
            // }
            // else {
            //     setPointIsNotReached = true;
            // }
            // if (setPointIsNotReached) {
            //     // obtain the encoder position
            //     encoderPosition = arm.getCurrentPosition();
            //     // calculate the error
            //     error = targetArm - encoderPosition;
            //     // rate of change of the error
            //     derivative = (error - lastError) / timer.seconds();
            //     // sum of all error over time
            //     integralSum = integralSum + (error * timer.seconds());
            //     armPower = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
            //     lastError = error;
            //     // reset the timer for next time
            //     timer.reset();
            // }
            // Send power to arm
            arm.setPower(armPower);
            arm.setTargetPosition(targetArm);
            // Send power to tower
            tower.setPower(towerPower);
            tower.setTargetPosition(targetTower);

            // Send telemetry data to the driver station
            // Controller telemetry
            telemetry.addLine("Controller telemetry:");
            telemetry.addData("Left Joystick Y", y);
            telemetry.addData("Left Joystick X", x);
            telemetry.addData("Right Joystick X", rx);
            telemetry.addData("front left motor power", frontLeftMotor.getPower());
            telemetry.addData("front right motor power", frontRightMotor.getPower());
            telemetry.addData("back left motor power", backLeftMotor.getPower());
            telemetry.addData("back right motor power", backRightMotor.getPower());
            telemetry.addData("left bump", gamepad1.left_bumper);
            telemetry.addData("Bucket Position", bucketTake ? "Take" : "Drop");
            // Claw telemetry
            telemetry.addLine("Claw telemetry:");
            telemetry.addData("Claw Position", clawOpen ? "Open" : "Closed");
            telemetry.addLine("Wrist telemetry:");
            telemetry.addData("Wrist Position", wristDown ? "Down" : "Up");
            // Arm telemetry
            telemetry.addLine("Arm telemetry:");
            telemetry.addData("Arm Power", arm.getPower());
            telemetry.addData("Dpad up", gamepad1.dpad_up);
//            telemetry.addData("Arm Target", targetArm);
            telemetry.addData("Arm Position", arm.getCurrentPosition());
            telemetry.addData("setPointIsNotReached", setPointIsNotReached);
            // Tower telemetry
            telemetry.addLine("Tower telemetry:");
            telemetry.addData("Tower Power", tower.getPower());
            telemetry.addData("Tower Position", tower.getCurrentPosition());
            telemetry.addData("Tower Target", targetTower);
            // var telemetry
            telemetry.addLine("Variable telemetry:");
            telemetry.addData("Tower Extended", towerExtended);
            telemetry.update();
        }
    }
}
