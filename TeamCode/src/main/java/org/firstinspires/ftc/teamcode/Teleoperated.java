package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@TeleOp(name="EviumMainTeleop", group="TeleOp")
public class Teleoperated extends OpMode {

    // Motors & Sensors
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotorEx cap;
    private DcMotorEx cap2;
    private DcMotor spindle;
    private SparkFunOTOS odometry;
    private RevTouchSensor Lswitch;

    // Servo Button Toggles
    private boolean LSLowerOut = true;
    private boolean LSLowerToggling = false;
    private double LSLowerPos;
    private boolean LSTopOut;
    private boolean LSTopToggling;
    private double vindoViper = 0;
    private boolean switchViperDirection = false;
    private double capPower;
    private Rev2mDistanceSensor frontDistance;
    private Rev2mDistanceSensor backDistance;
    private PIDController distancePID = new PIDController(0.1, 0, 0);

    // Servos
    private ServoImplEx LSLower;
    private ServoImplEx LSTop;

    @Override
    public void init() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftMotor = hardwareMap.get(DcMotor.class, "leftBack");
        backRightMotor = hardwareMap.get(DcMotor.class, "rightBack");

        spindle = hardwareMap.get(DcMotor.class, "spindle");
        odometry = hardwareMap.get(SparkFunOTOS.class, "odometry");
        cap = hardwareMap.get(DcMotorEx.class, "cap");
        cap2 = hardwareMap.get(DcMotorEx.class, "cap2");
        LSLower = hardwareMap.get(ServoImplEx.class, "LSLower");
        LSTop = hardwareMap.get(ServoImplEx.class, "LSTop");
        Lswitch = hardwareMap.get(RevTouchSensor.class, "Lswitch");
        frontDistance = hardwareMap.get(Rev2mDistanceSensor.class, "frontDistance");
        backDistance = hardwareMap.get(Rev2mDistanceSensor.class, "backDistance");


        // Define Servo range
        LSLower.setPwmEnable();
        LSTop.setPwmEnable();
        LSLower.scaleRange(0, 1);
        LSTop.scaleRange(0, 1);

        // Set motor directions
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set motor modes
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        cap.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cap2.setDirection(DcMotorSimple.Direction.REVERSE);
        spindle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Breaking mode
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cap.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Make sure odo is ready
        odometry.begin();
        odometry.setPosition(new SparkFunOTOS.Pose2D(0,0,0));
    }

    @Override
    public void start() {
        telemetry.speak("Evium Start");
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
    }

    @Override
    public void loop() {
        // Get joystick values
        double y;
        double x;
        double rx;

        if (gamepad1.right_trigger > 0.33) {
            y = -gamepad1.left_stick_y * 0.2;
            x = gamepad1.left_stick_x * 0.5;
            rx = gamepad1.right_stick_x * 0.25;
        } else if (gamepad1.a) {
            distancePID.setSetPoint(5.8);
            y = - distancePID.calculate(frontDistance.getDistance(DistanceUnit.INCH));
            x = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;
        } else {
            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;
        }

        // Set power to motors
        frontLeftMotor.setPower(y + x + rx);
        backLeftMotor.setPower(y - x + rx);
        frontRightMotor.setPower(y - x - rx);
        backRightMotor.setPower(y + x - rx);
        if (gamepad2.x) {
            LSLower.setPosition(0);
            LSTop.setPosition(1);
        }
        if (gamepad2.a) {
            LSLower.setPosition(0.77);
            LSTop.setPosition(0.23);
        }
        if (gamepad2.y) {
            LSLower.setPosition(0.4);
            LSTop.setPosition(0.6);
        }
        if ((gamepad2.b) || (gamepad1.b)) {
            if (vindoViper > 0.94) {
                switchViperDirection = true;
            }
            if (vindoViper < 0.06) {
                switchViperDirection = false;
            }
            if (switchViperDirection) {
                vindoViper = vindoViper - 0.05;
            } else {
                vindoViper = vindoViper + 0.05;
            }
            LSLower.setPosition(1 - vindoViper);
            LSTop.setPosition(1 - vindoViper);
        }
        if (cap.getCurrentPosition() > 675) {
            capPower = -0.25;
        } else {
            if (gamepad2.right_trigger > 0.05) {
                capPower = -gamepad2.right_trigger;
            } else {
                capPower = gamepad2.left_trigger;
            }
        }
        cap.setPower(capPower);
        cap2.setPower(capPower);
        if (Lswitch.isPressed()) {
            telemetry.speak("Warning Capstan. Pull Up");
            cap.setPower(capPower * 0.1);
            cap.setPower(capPower * 0.1);
        }
        spindle.setPower(-gamepad2.left_stick_y * 0.75);
        // Telemetry for debugging
        SparkFunOTOS.Pose2D pos = odometry.getPosition();
        telemetry.addData("DistanceFront", frontDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("OdoX", pos.x);
        telemetry.addData("OdoY", pos.y);
        telemetry.addData("Angle", pos.h);
        telemetry.addData("CapstanInDanger?", LSTop.getPosition());
        telemetry.update();
    }
}
