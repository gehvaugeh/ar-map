package com.example.pointcloudmapper.color

import android.media.Image
import com.google.ar.core.Camera
import com.example.pointcloudmapper.model.ColoredPoint
import java.nio.FloatBuffer
import kotlin.math.roundToInt

/**
 * Utility for extracting RGB color values from CPU YUV_420_888 images and projecting
 * ARCore 3D feature points into camera pixel space.
 */
object ColorProjectionUtils {

    /**
     * Extracts colored 3D points from ARCore PointCloud buffer using camera intrinsics and image buffer.
     *
     * @param pointCloudBuffer FloatBuffer from Frame.acquirePointCloud().points (4 floats per point: X, Y, Z, Confidence)
     * @param camera The ARCore Camera for current frame
     * @param image The YUV_420_888 Image acquired from Frame.acquireCameraImage()
     * @param minConfidence Minimum ARCore confidence score to accept point (0.0 to 1.0)
     */
    fun extractColoredPoints(
        pointCloudBuffer: FloatBuffer,
        camera: Camera,
        image: Image,
        minConfidence: Float = 0.1f
    ): List<ColoredPoint> {
        val coloredPoints = mutableListOf<ColoredPoint>()

        val imageIntrinsics = camera.imageIntrinsics
        val focalLength = imageIntrinsics.focalLength // fx, fy
        val principalPoint = imageIntrinsics.principalPoint // cx, cy
        val imageDimensions = imageIntrinsics.imageDimensions // width, height

        val imageWidth = imageDimensions[0]
        val imageHeight = imageDimensions[1]

        val yPlane = image.planes[0]
        val uPlane = image.planes[1]
        val vPlane = image.planes[2]

        val yBuffer = yPlane.buffer
        val uBuffer = uPlane.buffer
        val vBuffer = vPlane.buffer

        val yRowStride = yPlane.rowStride
        val uvRowStride = uPlane.rowStride
        val uvPixelStride = uPlane.pixelStride

        val pointsCount = pointCloudBuffer.remaining() / 4

        // Camera pose translation and rotation matrix
        val viewMatrix = FloatArray(16)
        camera.getViewMatrix(viewMatrix, 0)

        for (i in 0 until pointsCount) {
            val idx = i * 4
            val x = pointCloudBuffer.get(idx)
            val y = pointCloudBuffer.get(idx + 1)
            val z = pointCloudBuffer.get(idx + 2)
            val confidence = pointCloudBuffer.get(idx + 3)

            if (confidence < minConfidence) continue

            // Transform world point into camera coordinate system
            // View matrix is 4x4 column-major: P_cam = ViewMatrix * P_world
            val camX = viewMatrix[0] * x + viewMatrix[4] * y + viewMatrix[8] * z + viewMatrix[12]
            val camY = viewMatrix[1] * x + viewMatrix[5] * y + viewMatrix[9] * z + viewMatrix[13]
            val camZ = viewMatrix[2] * x + viewMatrix[6] * y + viewMatrix[10] * z + viewMatrix[14]

            // Check if point is in front of camera
            if (camZ >= 0) continue

            // Project 3D camera coordinates to pixel coordinates (X_cam, Y_cam, Z_cam)
            // ARCore camera coordinate system: X right, Y up, Z backwards (-Z is looking direction)
            // Pixel projection: u = cx + fx * (X / -Z), v = cy - fy * (Y / -Z)
            val depth = -camZ
            val pixelX = (principalPoint[0] + (focalLength[0] * (camX / depth))).roundToInt()
            val pixelY = (principalPoint[1] - (focalLength[1] * (camY / depth))).roundToInt()

            if (pixelX < 0 || pixelX >= imageWidth || pixelY < 0 || pixelY >= imageHeight) {
                continue
            }

            // Extract YUV color from image buffer at (pixelX, pixelY)
            val yIdx = pixelY * yRowStride + pixelX
            val uvIdx = (pixelY / 2) * uvRowStride + (pixelX / 2) * uvPixelStride

            if (yIdx >= yBuffer.capacity() || uvIdx >= uBuffer.capacity() || uvIdx >= vBuffer.capacity()) {
                continue
            }

            val yVal = (yBuffer.get(yIdx).toInt() and 0xFF).toFloat()
            val uVal = ((uBuffer.get(uvIdx).toInt() and 0xFF) - 128).toFloat()
            val vVal = ((vBuffer.get(uvIdx).toInt() and 0xFF) - 128).toFloat()

            // YUV to RGB conversion formula
            val r = (yVal + 1.370705f * vVal).roundToInt().coerceIn(0, 255)
            val g = (yVal - 0.337633f * uVal - 0.698001f * vVal).roundToInt().coerceIn(0, 255)
            val b = (yVal + 1.732446f * uVal).roundToInt().coerceIn(0, 255)

            coloredPoints.add(
                ColoredPoint(
                    x = x,
                    y = y,
                    z = z,
                    r = r,
                    g = g,
                    b = b,
                    confidence = confidence
                )
            )
        }

        return coloredPoints
    }
}
