package com.example.RobotArm

import kotlin.math.*

class RobotFK(private val model: RobotModel) {
    private val Debug_Tag = "RobotFK"
    init {
        require(model.dhParams.size == 6) {
            "dhParams must contain exactly 6 DH parameter sets."
        }
    }
    fun roundIfTiny(v: Float) = if (abs(v) < 1e-3f) 0f else (round(v * 1000f) / 1000f)

    fun roundDeg(v: Double): Float {
        val deg = Math.toDegrees(v)
        return if (abs(deg) < 1e-3) {
            0f
        }
        else (round(deg * 10000.0) / 10000f).toFloat()
    }

    // Standard DH matrix
    private fun dhMatrix(theta: Float, d: Float, a: Float, alpha: Float): Array<FloatArray> {
        val ct = cos(theta)
        val st = sin(theta)
        val ca = cos(alpha)
        val sa = sin(alpha)

        return arrayOf(
            floatArrayOf(ct, -st * ca, st * sa, a * ct),
            floatArrayOf(st, ct * ca, -ct * sa, a * st),
            floatArrayOf(0f, sa, ca, d),
            floatArrayOf(0f, 0f, 0f, 1f)
        )
    }

    // Multiply two 4x4 matrices
    private fun multiply4(a: Array<FloatArray>, b: Array<FloatArray>): Array<FloatArray> {
        val r = Array(4) { FloatArray(4) }
        for (i in 0..3) {
            for (j in 0..3) {
                var s = 0f
                for (k in 0..3) {
                    s += a[i][k] * b[k][j]
                }
                r[i][j] = s
            }
        }
        return r
    }

    // Apply base X offset (from RobotModel)
    private fun applyBaseOffsetX(T: Array<FloatArray>): Array<FloatArray> {
        val result = Array(4) { FloatArray(4) }
        for (i in 0..3) {
            for (j in 0..3) {
                result[i][j] = roundIfTiny(T[i][j])
            }
        }
        result[0][3] += model.baseXOffset
        return result
    }

    // Forward kinematics in radians
    fun forwardKinematics(thetaRad: FloatArray): Array<FloatArray> {
        require(thetaRad.size == 7) { "thetaRad must have length 7 (index 1..6 used)" }

        // Apply theta offsets
        val t = FloatArray(6)
        for (i in 0 until 6) {
            t[i] = thetaRad[i + 1] + model.dhParams[i].thetaOffset
        }


        // First transform
        var T = dhMatrix(
            t[0],
            model.dhParams[0].d,
            model.dhParams[0].a,
            model.dhParams[0].alpha
        )

        // MULTIPLY FROM T01 AND T12 UNTIL T06
        for (i in 1 until 6) {
            val p = model.dhParams[i]
            T = multiply4(T, dhMatrix(t[i], p.d, p.a, p.alpha))
        }

        // Apply flange transform
        val f = model.flange
        T = multiply4(T, dhMatrix(f.thetaOffset, f.d, f.a, f.alpha))

        return applyBaseOffsetX(T)
    }

    // Degrees version
    fun forwardKinematicsDeg(thetaDeg: FloatArray): Array<FloatArray> {
        require(thetaDeg.size == 7)
        val rad = FloatArray(7)
        for (i in 1..6) {
            val corrected = thetaDeg[i] * model.jointDirection[i]
            rad[i] = Math.toRadians(corrected.toDouble()).toFloat()
        }
        return forwardKinematics(rad)
    }

    // Extract XYZ
    fun getTCPPosition(T: Array<FloatArray>): FloatArray {
        return floatArrayOf(roundIfTiny(T[0][3]), roundIfTiny(T[1][3]), roundIfTiny(T[2][3])
        )
    }

    // Extract rotation 3x3
    fun   getTCPOrientation(T: Array<FloatArray>): Array<FloatArray> {
        return arrayOf(
            floatArrayOf(T[0][0], T[0][1], T[0][2]),
            floatArrayOf(T[1][0], T[1][1], T[1][2]),
            floatArrayOf(T[2][0], T[2][1], T[2][2])
        )
    }

    // Euler angles ZYX
    fun getTCPEulerZYX(R: Array<FloatArray>): FloatArray {
        val r20 = R[2][0].toDouble()
        val pitch = -asin(r20)
        val cosy = cos(pitch)

        val roll: Double
        val yaw: Double

        if (abs(cosy) > 1e-6) {
            roll = atan2(R[2][1].toDouble(), R[2][2].toDouble())
            yaw = atan2(R[1][0].toDouble(), R[0][0].toDouble())
        } else {
            roll = 0.0
            yaw = atan2(-R[0][1].toDouble(), R[1][1].toDouble())
        }

        return floatArrayOf(
            roundDeg(yaw),
            roundDeg(pitch),
            roundDeg(roll)
        )
    }

    // Euler X-Y-Z (intrinsic) extractor (degrees)
    fun getTCPEulerXYZ(R: Array<FloatArray>): FloatArray {
        val ex: Double
        val ey: Double
        val ez: Double

        // EY = asin(R20)
        ey = asin(R[2][0].toDouble())
        val cosy = cos(ey)

        if (abs(cosy) > 1e-6) {
            // EX = atan2(-R21, R22)
            ex = atan2(-R[2][1].toDouble(), R[2][2].toDouble())

            // EZ = atan2(-R10, R00)
            ez = atan2(-R[1][0].toDouble(), R[0][0].toDouble())
        } else {
            // gimbal lock
            ex = 0.0
            ez = atan2(R[0][1].toDouble(), R[1][1].toDouble())
        }

        return floatArrayOf(
            roundDeg(ex),
            roundDeg(ey),
            roundDeg(ez)
        )
    }

}
