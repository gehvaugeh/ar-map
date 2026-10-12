package com.example.pointcloudmapper.color

import android.media.Image
import com.example.pointcloudmapper.model.ColoredPoint
import com.google.ar.core.Camera
import com.google.ar.core.Frame
import com.google.ar.core.PointCloud
import com.google.ar.core.exceptions.NotYetAvailableException
import java.nio.ByteOrder
import java.nio.IntBuffer
import kotlin.math.roundToInt

/**
 * Utility for extracting RGB color values from CPU YUV_420_888 images and projecting
 * ARCore 3D feature points or Raw Depth 3D points into camera pixel space.
 */
object ColorProjectionUtils {

    /**
     * Extracts colored 3D points from ARCore Raw Depth Image if available, falling back to VIO PointCloud.
     */
    fun extractColoredPoints(
        frame: Frame,
        camera: Camera,
        yuvImage: Image
    ): List<ColoredPoint> {
        val coloredPoints = mutableListOf<ColoredPoint>()

        // Try extracting from Raw Depth API first (rock-solid geometric depth)
        try {
            val rawDepthImage = frame.acquireRawDepthImage16Bits()
            val rawDepthConfidenceImage = frame.acquireRawDepthConfidenceImage()

            if (rawDepthImage != null && rawDepthConfidenceImage != null) {
                val depthPoints = extractPointsFromRawDepth(
                    rawDepthImage = rawDepthImage,
                    rawDepthConfidenceImage = rawDepthConfidenceImage,
                    camera = camera,
                    yuvImage = yuvImage
                )
                rawDepthImage.close()
                rawDepthConfidenceImage.close()

                if (depthPoints.isNotEmpty()) {
                    return depthPoints
                }
            }
        } catch (e: NotYetAvailableException) {
            // Depth frame not available yet, fall back to PointCloud
        } catch (e: Exception) {
            // Fall back to PointCloud
        }

        // Fallback: VIO Feature PointCloud
        try {
            val pointCloud = frame.acquirePointCloud()
            val pointCloudPoints = extractFromPointCloud(
                pointCloud = pointCloud,
                camera = camera,
                image = yuvImage,
                minConfidence = 0.3f
            )
            pointCloud.release()
            return pointCloudPoints
        } catch (e: Exception) {
            // Ignore
        }

        return coloredPoints
    }

    private fun extractPointsFromRawDepth(
        rawDepthImage: Image,
        rawDepthConfidenceImage: Image,
        camera: Camera,
        yuvImage: Image
    ): List<ColoredPoint> {
        val points = mutableListOf<ColoredPoint>()

        val depthWidth = rawDepthImage.width
        val depthHeight = rawDepthImage.height

        val depthBuffer = rawDepthImage.planes[0].buffer.order(ByteOrder.LITTLE_ENDIAN).asShortBuffer()
        val confidenceBuffer = rawDepthConfidenceImage.planes[0].buffer

        val imageIntrinsics = camera.imageIntrinsics
        val focalLength = imageIntrinsics.focalLength
        val principalPoint = imageIntrinsics.principalPoint
        val imageDimensions = imageIntrinsics.imageDimensions

        val yuvWidth = imageDimensions[0]
        val yuvHeight = imageDimensions[1]

        val yPlane = yuvImage.planes[0]
        val uPlane = yuvImage.planes[1]
        val vPlane = yuvImage.planes[2]

        val yBuffer = yPlane.buffer
        val uBuffer = uPlane.buffer
        val vBuffer = vPlane.buffer

        val yRowStride = yPlane.rowStride
        val uvRowStride = uPlane.rowStride
        val uvPixelStride = uPlane.pixelStride

        val pose = camera.pose

        // Subsample depth grid for efficiency (e.g. step = 6 pixels)
        val step = 6

        for (y in 0 until depthHeight step step) {
            for (x in 0 until depthWidth step step) {
                val depthIdx = y * depthWidth + x
                if (depthIdx >= depthBuffer.capacity()) continue

                val depthMm = depthBuffer.get(depthIdx).toInt() and 0xFFFF
                if (depthMm <= 200 || depthMm > 5000) continue // Filter depth range 0.2m - 5.0m

                val confidence = if (depthIdx < confidenceBuffer.capacity()) {
                    (confidenceBuffer.get(depthIdx).toInt() and 0xFF) / 255.0f
                } else {
                    1.0f
                }

                if (confidence < 0.4f) continue // High confidence depth filter

                val depthMeters = depthMm / 1000.0f

                // Project Depth Pixel (x, y) to Camera 3D Coordinates (X_cam, Y_cam, Z_cam)
                // Intrinsics scaling for depth image resolution relative to RGB image resolution
                val scaleX = yuvWidth.toFloat() / depthWidth.toFloat()
                val scaleY = yuvHeight.toFloat() / depthHeight.toFloat()

                val rgbPixelX = x * scaleX
                val rgbPixelY = y * scaleY

                val camX = (rgbPixelX - principalPoint[0]) * depthMeters / focalLength[0]
                val camY = -(rgbPixelY - principalPoint[1]) * depthMeters / focalLength[1]
                val camZ = -depthMeters

                // Transform 3D point from Camera Space to World Space
                val worldPoint = pose.transformPoint(floatArrayOf(camX, camY, camZ))

                // Extract YUV color at (rgbPixelX, rgbPixelY)
                val px = rgbPixelX.roundToInt().coerceIn(0, yuvWidth - 1)
                val py = rgbPixelY.roundToInt().coerceIn(0, yuvHeight - 1)

                val yIdx = py * yRowStride + px
                val uvIdx = (py / 2) * uvRowStride + (px / 2) * uvPixelStride

                if (yIdx >= yBuffer.capacity() || uvIdx >= uBuffer.capacity() || uvIdx >= vBuffer.capacity()) {
                    continue
                }

                val yVal = (yBuffer.get(yIdx).toInt() and 0xFF).toFloat()
                val uVal = ((uBuffer.get(uvIdx).toInt() and 0xFF) - 128).toFloat()
                val vVal = ((vBuffer.get(uvIdx).toInt() and 0xFF) - 128).toFloat()

                val r = (yVal + 1.370705f * vVal).roundToInt().coerceIn(0, 255)
                val g = (yVal - 0.337633f * uVal - 0.698001f * vVal).roundToInt().coerceIn(0, 255)
                val b = (yVal + 1.732446f * uVal).roundToInt().coerceIn(0, 255)

                val pointId = (worldPoint[0] * 100).toInt() hashWith (worldPoint[1] * 100).toInt() hashWith (worldPoint[2] * 100).toInt()

                points.add(
                    ColoredPoint(
                        id = pointId,
                        x = worldPoint[0],
                        y = worldPoint[1],
                        z = worldPoint[2],
                        r = r,
                        g = g,
                        b = b,
                        confidence = confidence
                    )
                )
            }
        }

        return points
    }

    private infix fun Int.hashWith(other: Int): Int {
        return this * 31 + other
    }

    private fun extractFromPointCloud(
        pointCloud: PointCloud,
        camera: Camera,
        image: Image,
        minConfidence: Float
    ): List<ColoredPoint> {
        val coloredPoints = mutableListOf<ColoredPoint>()

        val pointCloudBuffer = pointCloud.points
        val idBuffer: IntBuffer = pointCloud.ids

        val imageIntrinsics = camera.imageIntrinsics
        val focalLength = imageIntrinsics.focalLength
        val principalPoint = imageIntrinsics.principalPoint
        val imageDimensions = imageIntrinsics.imageDimensions

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

        val viewMatrix = FloatArray(16)
        camera.getViewMatrix(viewMatrix, 0)

        for (i in 0 until pointsCount) {
            val idx = i * 4
            val x = pointCloudBuffer.get(idx)
            val y = pointCloudBuffer.get(idx + 1)
            val z = pointCloudBuffer.get(idx + 2)
            val confidence = pointCloudBuffer.get(idx + 3)

            val pointId = if (idBuffer.hasRemaining()) idBuffer.get(i) else i

            if (confidence < minConfidence) continue

            val camX = viewMatrix[0] * x + viewMatrix[4] * y + viewMatrix[8] * z + viewMatrix[12]
            val camY = viewMatrix[1] * x + viewMatrix[5] * y + viewMatrix[9] * z + viewMatrix[13]
            val camZ = viewMatrix[2] * x + viewMatrix[6] * y + viewMatrix[10] * z + viewMatrix[14]

            if (camZ >= 0) continue

            val depth = -camZ
            val pixelX = (principalPoint[0] + (focalLength[0] * (camX / depth))).roundToInt()
            val pixelY = (principalPoint[1] - (focalLength[1] * (camY / depth))).roundToInt()

            if (pixelX < 0 || pixelX >= imageWidth || pixelY < 0 || pixelY >= imageHeight) {
                continue
            }

            val yIdx = pixelY * yRowStride + pixelX
            val uvIdx = (pixelY / 2) * uvRowStride + (pixelX / 2) * uvPixelStride

            if (yIdx >= yBuffer.capacity() || uvIdx >= uBuffer.capacity() || uvIdx >= vBuffer.capacity()) {
                continue
            }

            val yVal = (yBuffer.get(yIdx).toInt() and 0xFF).toFloat()
            val uVal = ((uBuffer.get(uvIdx).toInt() and 0xFF) - 128).toFloat()
            val vVal = ((vBuffer.get(uvIdx).toInt() and 0xFF) - 128).toFloat()

            val r = (yVal + 1.370705f * vVal).roundToInt().coerceIn(0, 255)
            val g = (yVal - 0.337633f * uVal - 0.698001f * vVal).roundToInt().coerceIn(0, 255)
            val b = (yVal + 1.732446f * uVal).roundToInt().coerceIn(0, 255)

            coloredPoints.add(
                ColoredPoint(
                    id = pointId,
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
