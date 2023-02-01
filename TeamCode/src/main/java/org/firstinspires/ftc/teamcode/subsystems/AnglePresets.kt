package org.firstinspires.ftc.teamcode.subsystems

object AnglePresets {
    @JvmField var aGROUND = -50
    @JvmField var bLOW = 0
    @JvmField var cMID = 40
    @JvmField var dHIGH = 85
    @JvmField var eBACKHIGH = 100
    @JvmField var fBACKMID = 150
    @JvmField var gBACKLOW = 180
    @JvmField var hBACKGROUND = 220

    @JvmField var i5 = -30
    @JvmField var j4 = -35
    @JvmField var k3 = -40
    @JvmField var l2 = -45

    val GROUND get() = aGROUND
    val LOW get() = bLOW
    val MID get() = cMID
    val HIGH get() = dHIGH
    val BACKHIGH get() = eBACKHIGH
    val BACKMID get() = fBACKMID
    val BACKLOW get() = gBACKLOW
    val BACKGROUND get() = hBACKGROUND

    val STACK get() = i5

    val all = listOf(aGROUND, bLOW, cMID, dHIGH, eBACKHIGH, fBACKMID, gBACKLOW, hBACKGROUND)

    fun next(angle: Int): Int {
        val idx = all.indexOf(angle)
        if (idx == -1 || idx == all.size - 1) return angle
        return all[idx + 1]
    }

    fun prev(angle: Int): Int {
        val idx = all.indexOf(angle)
        if (idx == -1 || idx == 0) return angle
        return all[idx - 1]
    }
}
