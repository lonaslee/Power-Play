package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.robotcore.external.Telemetry

class Claw(
    hardwareMap: HardwareMap,
    private val gamepads: Pair<GamepadExt, GamepadExt>? = null,
    private val telemetry: Telemetry? = null,
    private val arm: Arm? = null
) : Subsystem {
    constructor(hardwareMap: HardwareMap, telemetry: Telemetry, arm: Arm? = null) : this(
        hardwareMap, null, telemetry, arm
    )

    private val servo = (hardwareMap.get(RobotConfig.CLAW.s) as ServoImplEx).apply {
        direction = Servo.Direction.FORWARD
    }

    companion object {
        const val OPEN_POS = 0.56
        const val CLOSE_POS = 0.69
        const val HALF_OPEN_POS = 0.6
    }

    override val state
        get() = when (servo.position) {
            in OPEN_POS - 0.01..OPEN_POS + 0.01   -> OPEN_POS
            in CLOSE_POS - 0.01..CLOSE_POS + 0.01 -> CLOSE_POS
            else                                  -> HALF_OPEN_POS
        }

    fun open() {
        servo.position = OPEN_POS
    }

    fun halfOpen() {
        servo.position = HALF_OPEN_POS
    }

    fun close() {
        servo.position = CLOSE_POS
    }

    fun change() {
        if (state == CLOSE_POS) open()
        else close()
    }

    private fun updateToArm() {
        arm?.let {
            if (state == OPEN_POS && arm.state > Arm.Height.MID) halfOpen()
            else if (state == HALF_OPEN_POS && arm.state <= Arm.Height.MID) open()
        }
    }

    fun update(gp1: GamepadExt, gp2: GamepadExt) {
        telemetry?.addData("claw pos", ::state.name)
        if (anypressed(gp1::left_bumper, gp2::left_bumper)) change()
        updateToArm()
    }

    override fun update() {
        if (gamepads == null) updateToArm() else update(gamepads.first, gamepads.second)
    }
}
