package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.hardware.Gamepad
import kotlin.jvm.internal.CallableReference
import kotlin.reflect.KProperty
import kotlin.reflect.jvm.javaField

/**
 * Subclass wrapper of [Gamepad] with helper functions
 */
class GamepadExt(private val gamepad: Gamepad) : Gamepad() {
    val prev = Gamepad()

    /**
     * Update the fields of this object with the injected [Gamepad] reference.
     */
    fun sync() = prev.copy(this).also { this.copy(gamepad) }
}

/* * * * * * * * * * * *\
 * GamepadExt helpers  *
\* * * * * * * * * * * */

/**
 * Pair of gamepad1 to gamepad2.
 */
typealias Gamepads = Pair<GamepadExt, GamepadExt>

/**
 * [GamepadExt.sync] on both of the pair.
 */
fun Gamepads.sync() {
    first.sync()
    second.sync()
}

/**
 * Return if a Boolean button has been pressed.
 */
fun pressed(button: KProperty<*>): Boolean {
    val gamepad = (button as CallableReference).boundReceiver as GamepadExt
    return button.javaField!!.let { !it.getBoolean(gamepad.prev) && it.getBoolean(gamepad) }
}

/**
 * Return if a Boolean button has been released.
 */
fun released(button: KProperty<*>): Boolean {
    val gamepad = (button as CallableReference).boundReceiver as GamepadExt
    return button.javaField!!.let { it.getBoolean(gamepad.prev) && !it.getBoolean(gamepad) }
}

/**
 * Return if a Float trigger or joystick axis has been moved away from zero.
 */
fun moved(triggerOrJoystick: KProperty<*>): Boolean {
    val gamepad = (triggerOrJoystick as CallableReference).boundReceiver as GamepadExt
    return triggerOrJoystick.javaField!!.let {
        it.getFloat(gamepad.prev) == 0F && it.getFloat(
            gamepad
        ) != 0F
    }
}
