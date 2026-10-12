package com.example.pointcloudmapper

import com.example.pointcloudmapper.export.PlyExporter
import com.example.pointcloudmapper.model.ColoredPoint
import com.example.pointcloudmapper.spatial.VoxelGridPointCloudManager
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import java.io.File

class PointCloudMapperUnitTest {

    @Test
    fun testPointTrackingById() {
        val manager = VoxelGridPointCloudManager(voxelSize = 0.05f)

        // Point with ID 100 initially observed at position (0.01, 0.01, 0.01) with confidence 0.5
        val p1 = ColoredPoint(id = 100, x = 0.01f, y = 0.01f, z = 0.01f, r = 255, g = 0, b = 0, confidence = 0.5f)
        // Point with ID 100 refined by ARCore SLAM to position (0.012, 0.011, 0.010) with higher confidence 0.9
        val p2 = ColoredPoint(id = 100, x = 0.012f, y = 0.011f, z = 0.010f, r = 0, g = 255, b = 0, confidence = 0.9f)
        // Point with ID 101 at different location
        val p3 = ColoredPoint(id = 101, x = 0.06f, y = 0.06f, z = 0.06f, r = 0, g = 0, b = 255, confidence = 0.8f)

        manager.addPoint(p1)
        assertEquals(1, manager.pointCount())

        manager.addPoint(p2)
        assertEquals(1, manager.pointCount()) // Updated point 100
        assertEquals(0.012f, manager.getPoints()[0].x, 0.0001f)
        assertEquals(255, manager.getPoints()[0].g)

        manager.addPoint(p3)
        assertEquals(2, manager.pointCount())
    }

    @Test
    fun testPlyExporterFormat() {
        val tempFile = File.createTempFile("test_pointcloud", ".ply")
        tempFile.deleteOnExit()

        val points = listOf(
            ColoredPoint(id = 1, x = 1.0f, y = 2.0f, z = 3.0f, r = 255, g = 128, b = 64),
            ColoredPoint(id = 2, x = -0.5f, y = 0.0f, z = 1.2f, r = 0, g = 255, b = 0)
        )

        PlyExporter.exportToAsciiPly(points, tempFile)

        val lines = tempFile.readLines()
        assertEquals("ply", lines[0])
        assertEquals("format ascii 1.0", lines[1])
        assertTrue(lines.contains("element vertex 2"))
        assertTrue(lines.contains("property float x"))
        assertTrue(lines.contains("property float y"))
        assertTrue(lines.contains("property float z"))
        assertTrue(lines.contains("property uchar red"))
        assertTrue(lines.contains("property uchar green"))
        assertTrue(lines.contains("property uchar blue"))
        assertEquals("end_header", lines[lines.indexOf("end_header")])

        val lastTwoLines = lines.takeLast(2)
        assertEquals("1.0 2.0 3.0 255 128 64", lastTwoLines[0])
        assertEquals("-0.5 0.0 1.2 0 255 0", lastTwoLines[1])
    }
}
