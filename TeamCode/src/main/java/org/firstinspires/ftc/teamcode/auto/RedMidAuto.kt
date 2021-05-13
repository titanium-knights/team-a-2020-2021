package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.util.Intake
import org.firstinspires.ftc.teamcode.util.WobbleGoal
import java.lang.Math.toRadians

class RedMidAuto: LinearOpMode() {
    override fun runOpMode() {
        val drive = SampleMecanumDrive(hardwareMap)
        val wobbleGoal = WobbleGoal.standard(hardwareMap)

        val intake = Intake.standard(hardwareMap)
        val shooterMotor = hardwareMap.dcMotor["shooter1"]
        shooterMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        val shooterMotor2 = hardwareMap.dcMotor["shooter2"]
        shooterMotor2.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        val shooterServo = hardwareMap.servo["pinball"]
        val flap = hardwareMap.servo["Flap"]

        // init stuff
        wobbleGoal.liftArm()
        wobbleGoal.grab()
        flap.position = 0.00 //High goal: 0.00      PS: 0.05
        shooterServo.position = 0.00

        waitForStart()

        val box = 0 // TODO: Vision stuff

        val startPose = Pose2d(-64.0, -24.0, toRadians(180.0))
        drive.poseEstimate = startPose

        val rightPowerShotTrajectory = drive.trajectoryBuilder(startPose, true)
                .splineToConstantHeading(Vector2d(-3.0, -12.5), toRadians(0.0))
                .build()
        val midPowerShotTrajectory = drive.trajectoryBuilder(rightPowerShotTrajectory.end(), true)
                .strafeLeft(8.0)
                .build()
        val leftPowerShotTrajectory = drive.trajectoryBuilder(midPowerShotTrajectory.end(), true)
                .strafeLeft(7.0)
                .build()
        drive.followTrajectory(rightPowerShotTrajectory)
        drive.followTrajectory(midPowerShotTrajectory)
        drive.followTrajectory(leftPowerShotTrajectory)

        lateinit var trajectory: Trajectory

        when (box) {
            0 -> {
                trajectory = drive.trajectoryBuilder(leftPowerShotTrajectory.end(), toRadians(0.0))
                        .splineToLinearHeading(Pose2d(12.0, -41.0, toRadians(90.0)), toRadians(-90.0))
                        .build()
                drive.followTrajectory(trajectory)
            }
            1 -> {
                trajectory = drive.trajectoryBuilder(leftPowerShotTrajectory.end(), toRadians(0.0))
                        .splineToConstantHeading(Vector2d(16.0, -36.0), toRadians(90.0))
                        .build()
                drive.followTrajectory(trajectory)
            }
        }

        val backup = drive.trajectoryBuilder(trajectory.end(), false)
                .splineToLinearHeading(Pose2d(-30.0, -26.0, toRadians(0.0)), toRadians(180.0)) //pick up wobble2
                .build()
        drive.followTrajectory(backup)
    }
}