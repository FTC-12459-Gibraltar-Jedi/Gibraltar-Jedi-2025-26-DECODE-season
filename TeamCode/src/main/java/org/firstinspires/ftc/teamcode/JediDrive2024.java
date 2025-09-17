package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "JediDrive")
public class JediDrive2024 extends LinearOpMode {

    private DcMotor FrontLeft;
    private DcMotor BackLeft;
    private DcMotor FrontRight;
    private DcMotor BackRight;

    private DcMotor LeftTower;
    private DcMotor RightTower;
    private DcMotor OutArm;
    private DcMotor Hanger;

    private Servo UpDownWrist;
    private Servo LeftRightWrist;
    private CRServo Intake;

    private boolean UpDownWristToggled;
    private boolean G2UpDown;

    private boolean moveModeManual;
    private boolean moveModePressed;
    private int moveModePos;

    private boolean fieldOriented;
    private boolean fieldOrientedPressed;

    private double fieldOrientAngleOffset;

    // Lights
    private Servo Headlight;
    private Servo RGBLight;

    // Limelight!!
    // private Limelight3A limelight;

    // Control hub gyroscope
    private IMU imu;
    private double robotOrientation;

    public double convertAngle(double angle) {
        if (angle < 0) {
            angle += 360;
        }
        return angle % 360;
    }

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double Power;
        int Alt;
        int numOfPresses;
        int numOfPresses2;
        int tankDrive = 0;
        double runningError = 0;

        FrontLeft = hardwareMap.get(DcMotor.class, "Front Left");
        BackLeft = hardwareMap.get(DcMotor.class, "Back Left");
        FrontRight = hardwareMap.get(DcMotor.class, "Front Right");
        BackRight = hardwareMap.get(DcMotor.class, "Back Right");
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Setup Towers
        LeftTower = hardwareMap.get(DcMotor.class, "LeftTower");
        RightTower = hardwareMap.get(DcMotor.class, "RightTower");
        LeftTower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightTower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Other
        OutArm = hardwareMap.get(DcMotor.class, "OutArm");
        Hanger = hardwareMap.get(DcMotor.class, "Hanger");
        OutArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Hanger.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Servos:
        UpDownWristToggled = true;
        G2UpDown = false;
        UpDownWrist = hardwareMap.get(Servo.class, "UpDownWrist");
        LeftRightWrist = hardwareMap.get(Servo.class,"LeftRightWrist");
        Intake = hardwareMap.get(CRServo.class, "Intake");

        Headlight = hardwareMap.get(Servo.class, "Headlight");
        RGBLight = hardwareMap.get(Servo.class, "RGBLight");

        moveModeManual = false;
        moveModePressed = false;
        moveModePos = 0;

        fieldOriented = false;
        fieldOrientedPressed = false;

        // LeftTower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // RightTower.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Limelight boilerplate
        /* limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();
        telemetry.update(); */

        // Control hub setup
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize( // See https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html#orthogonal-mounting
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );

        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            Alt = 1;
            numOfPresses = 0;
            numOfPresses2 = 0;
            // Put run blocks here.
            while (opModeIsActive()) {
                // Robot angle (yaw) between -180 and 180
                robotOrientation = convertAngle(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                telemetry.addData("Robot angle (yaw): ", robotOrientation);

                // On the entire match
                Headlight.setPosition(0.5);
                RGBLight.setPosition(0.480);

                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                /* limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

                // Reset IMU

                // Limelight
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    Pose3D fieldPos = result.getBotpose_MT2();
                    Pose3D relativePose = result.getBotpose();

                    telemetry.addData("MT2 Botpose: ", fieldPos);
                    telemetry.addData("simple pose: ", relativePose);
                    runningError = convertAngle(convertAngle(relativePose.getOrientation().getYaw()) - robotOrientation);
                } */


                // Speed Variables
                if (gamepad1.left_bumper) {
                    Power = .35;
                } else if (gamepad1.right_bumper) {
                    Power = 1;
                } else {
                    Power = 0.75;
                }
                // Drive Mode Select
                // Tank Drive selected by Default
                /* if (gamepad1.a || tankDrive == 1) {
                    tankDrive = 1;
                    Alt = 0;
                }
                if (gamepad1.y || Alt == 1) {
                    Alt = 1;
                    tankDrive = 0;
                } */
                if (tankDrive == 1) {
                    // Tank Drive Code
                    if (gamepad1.dpad_right) {
                        // Crab Right
                        FrontLeft.setPower(-Power);
                        BackLeft.setPower(Power);
                        FrontRight.setPower(-Power);
                        BackRight.setPower(Power);
                    } else if (gamepad1.dpad_left) {
                        // Crab Left
                        FrontLeft.setPower(Power);
                        BackLeft.setPower(-Power);
                        FrontRight.setPower(Power);
                        BackRight.setPower(-Power);
                    } else {
                        // Drive
                        FrontLeft.setPower(Power * gamepad1.left_stick_y);
                        BackLeft.setPower(Power * gamepad1.left_stick_y);
                        FrontRight.setPower(-(Power * gamepad1.right_stick_y));
                        BackRight.setPower(-(Power * gamepad1.right_stick_y));
                    }
                    if (gamepad1.dpad_down) {
                        FrontLeft.setPower(Power * 1);
                        BackLeft.setPower(Power * 1);
                        FrontRight.setPower(-(Power * 1));
                        BackRight.setPower(-(Power * 1));
                    }
                    if (gamepad1.dpad_up) {
                        FrontLeft.setPower(Power * -1);
                        BackLeft.setPower(Power * -1);
                        FrontRight.setPower(-(Power * -1));
                        BackRight.setPower(-(Power * -1));
                    }
                }
                if (Alt == 1) {
                    // Alternative Drive Code

                    // Control hub gyroscope adjustment
                    double x = gamepad1.left_stick_x;
                    double y = gamepad1.left_stick_y;

                    double r = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

                    double theta = Math.toDegrees(Math.atan2(y, x));

                    // Note: Add constant to roboOrient. to dictate which way is forward
                    if (fieldOriented) {
                        theta += robotOrientation + runningError;
                    }

                    if (theta < 0) { theta += 360; }

                    x = r * Math.cos(Math.toRadians(theta));
                    y = r * Math.sin(Math.toRadians(theta));

                    FrontLeft.setPower(Power * (-gamepad1.right_stick_x + (y - x)));
                    BackLeft.setPower(Power * (-gamepad1.right_stick_x + y + x));
                    FrontRight.setPower(-(Power * (gamepad1.right_stick_x + y + x)));
                    BackRight.setPower(-(Power * (gamepad1.right_stick_x + (y - x))));

                    /* if (gamepad1.dpad_down) {
                        FrontLeft.setPower(Power * 1);
                        BackLeft.setPower(Power * 1);
                        FrontRight.setPower(-(Power * 1));
                        BackRight.setPower(-(Power * 1));
                    }
                    else if (gamepad1.dpad_up) {
                        FrontLeft.setPower(Power * -1);
                        BackLeft.setPower(Power * -1);
                        FrontRight.setPower(-(Power * -1));
                        BackRight.setPower(-(Power * -1));
                    }
                    else if (gamepad1.dpad_left) {
                        FrontLeft.setPower(Power * 1);
                        BackLeft.setPower(-(Power * 1));
                        FrontRight.setPower(Power * 1);
                        BackRight.setPower(-(Power * 1));
                    }
                    else if (gamepad1.dpad_right) {
                        FrontLeft.setPower(-(Power * 1));
                        BackLeft.setPower(Power * 1);
                        FrontRight.setPower(-(Power * 1));
                        BackRight.setPower(Power * 1);
                    } */
                }


                // Extend towers upwards
                if (moveModeManual) {
                    LeftTower.setTargetPosition(moveModePos);
                    RightTower.setTargetPosition(-moveModePos);

                    LeftTower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    RightTower.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    LeftTower.setPower(0.5);
                    RightTower.setPower(0.5);

                    if (gamepad2.right_stick_y < -0.7) {
                        moveModePos += 30;
                    } else if (gamepad2.right_stick_y > 0.70) {
                        moveModePos -= 30;
                    }

                    // if (moveModePos > 5990) { moveModePos = 5990; }
                    // else if (moveModePos < 10) { moveModePos = 10; }
                }
                else {
                    // 160 rotations per inch here
                    if (gamepad2.a) {
                        LeftTower.setTargetPosition(4);
                        RightTower.setTargetPosition(-4);

                        LeftTower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        RightTower.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        LeftTower.setPower(1);
                        RightTower.setPower(1);
                    } else if (gamepad2.y) {

                        LeftTower.setTargetPosition(5990);
                        RightTower.setTargetPosition(-5990);

                        LeftTower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        RightTower.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        LeftTower.setPower(1);
                        RightTower.setPower(1);

                    } else if (gamepad2.x) {
                        LeftTower.setTargetPosition(1840);
                        RightTower.setTargetPosition(-1840);

                        LeftTower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        RightTower.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        LeftTower.setPower(1);
                        RightTower.setPower(1);

                    } else if (gamepad2.b) {

                        LeftTower.setTargetPosition(480);
                        RightTower.setTargetPosition(-480);

                        LeftTower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        RightTower.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        LeftTower.setPower(1);
                        RightTower.setPower(1);
                    }
                }

                /* int difference = LeftTower.getCurrentPosition() + RightTower.getCurrentPosition();
                double curve = Math.abs((((double)difference) / 200.0f) * 0.3);
                boolean isMovingUp = LeftTower.getCurrentPosition() < LeftTower.getTargetPosition();
                if (difference > 0) { // LeftTower is above RightTower
                    if (isMovingUp) {
                        LeftTower.setPower(moveModeManual ? (0.5 - curve) : (1 - curve));
                        RightTower.setPower(moveModeManual ? 0.5 : 1);
                    } else {
                        LeftTower.setPower(moveModeManual ? 0.5 : 1);
                        RightTower.setPower(moveModeManual ? (0.5 - curve) : (1 - curve));
                    }
                } else if (difference < 0) { // RightTower is above LeftTower
                    if (isMovingUp) {
                        LeftTower.setPower(moveModeManual ? 0.5 : 1);
                        RightTower.setPower(moveModeManual ? (0.5 - curve) : (1 - curve));
                    } else {
                        LeftTower.setPower(moveModeManual ? (0.5 - curve) : (1 - curve));
                        RightTower.setPower(moveModeManual ? 0.3 : 0.7);
                    }
                } */

                LeftRightWrist.setPosition((-gamepad2.left_stick_x * 0.26) + 0.26);

                if (gamepad2.dpad_up) {
                    OutArm.setPower(-1);
                } else if (gamepad2.dpad_down) {
                    OutArm.setPower(1);
                } else {
                    OutArm.setPower(0);
                }

                if (gamepad1.dpad_up) {
                    Hanger.setPower(1);
                } else if (gamepad1.dpad_down) {
                    Hanger.setPower(-1);
                } else {
                    Hanger.setPower(0);
                }


                // UpDown wrist
                if (!G2UpDown && gamepad2.right_bumper) {
                    UpDownWristToggled = !UpDownWristToggled;
                }
                G2UpDown = gamepad2.right_bumper;
                UpDownWrist.setPosition(UpDownWristToggled ? 0.46 : 0);

                // Swap move modes
                if (!moveModePressed && gamepad2.left_bumper) {
                    moveModeManual = !moveModeManual;
                    if (moveModeManual) {
                        moveModePos = LeftTower.getCurrentPosition();
                    }
                }
                moveModePressed = gamepad2.left_bumper;


                // Toggle field oriented mode g1 dpad_up
                if (!fieldOrientedPressed && gamepad1.dpad_up) {
                    fieldOriented = !fieldOriented;
                }
                fieldOrientedPressed = gamepad1.dpad_up;


                if (gamepad2.right_trigger >= 0.25) {
                    Intake.setPower(-1);
                } else if (gamepad2.left_trigger >= 0.25) {
                    Intake.setPower(1);
                } else {
                    Intake.setPower(0);
                }

                if (gamepad2.back) {
                    LeftTower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    RightTower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }

                // Reset gyro
                if (gamepad1.dpad_down) {
                    imu.resetYaw();
                }

                telemetry.addData("Left tower encoder", LeftTower.getCurrentPosition());
                telemetry.addData("Right tower encoder", RightTower.getCurrentPosition());
                telemetry.addData("difference in towers", LeftTower.getCurrentPosition() + RightTower.getCurrentPosition());

                if (moveModeManual) {
                    telemetry.addData("!! Towers Manual Mode On !! - Position:", moveModePos);
                }

                telemetry.update();
                // Put loop blocks here.
            }
        }
    }
}