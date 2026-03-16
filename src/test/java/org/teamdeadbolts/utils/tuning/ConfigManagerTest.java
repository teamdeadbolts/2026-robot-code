/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.utils.tuning;

import static org.junit.jupiter.api.Assertions.*;

import java.io.IOException;
import java.nio.file.*;
import org.junit.jupiter.api.*;

public class ConfigManagerTest {
    private Path tempDir;

    @BeforeEach
    public void setup()
            throws IOException, NoSuchFieldException, SecurityException, IllegalArgumentException,
                    IllegalAccessException {
        tempDir = Files.createTempDirectory("configtest_");
        var instanceField = ConfigManager.class.getDeclaredField("INSTANCE");
        instanceField.setAccessible(true);
        instanceField.set(null, null);
    }

    @AfterEach
    public void cleanup() throws IOException {
        if (tempDir != null && Files.exists(tempDir)) {
            Files.walk(tempDir).sorted((a, b) -> b.compareTo(a)).forEach(p -> {
                try {
                    Files.deleteIfExists(p);
                } catch (IOException ignored) {
                }
            });
        }
    }

    @Test
    public void testSaveAndLoadVersions() throws IOException {
        ConfigManager manager = ConfigManager.getTestInstance(tempDir);

        manager.set("drive_kp", 0.12);
        manager.set("drive_ki", 0.02);
        int[] versionsAfterFirst = manager.getVersions();
        assertEquals(1, versionsAfterFirst.length);
        assertEquals(1, versionsAfterFirst[0]);

        manager.saveCurrentVersion();

        manager.saveNewVersion();
        manager.set("drive_kp", 0.5);
        int[] versionsAfterSecond = manager.getVersions();
        assertEquals(2, versionsAfterSecond.length);

        Path currentLink = tempDir.resolve("current.json");
        assertTrue(Files.isSymbolicLink(currentLink));
        Path target = Files.readSymbolicLink(currentLink);
        assertEquals("v2.json", target.toString());

        manager.loadVersion(1);
        manager.debug();
        assertEquals(0.12, (double) manager.get("drive_kp"));

        Path newTarget = Files.readSymbolicLink(currentLink);
        assertEquals("v1.json", newTarget.toString());
    }

    @Test
    public void testGetVersionsEmpty() {
        ConfigManager manager = ConfigManager.getTestInstance(tempDir);
        int[] versions = manager.getVersions();
        assertEquals(1, versions.length);
    }

    @Test
    public void testSetAndGetDifferentTypes() {
        ConfigManager manager = ConfigManager.getTestInstance(tempDir);

        manager.set("doubleValue", 3.14);
        manager.set("intValue", 42);
        manager.set("stringValue", "hello");
        manager.set("booleanValue", true);

        Object doubleVal = manager.get("doubleValue");
        Object intVal = manager.get("intValue");
        Object strVal = manager.get("stringValue");
        Object boolVal = manager.get("booleanValue");

        assertTrue(doubleVal instanceof Double);
        assertTrue(intVal instanceof Integer);
        assertTrue(strVal instanceof String);
        assertTrue(boolVal instanceof Boolean);

        assertEquals(3.14, (Double) doubleVal, 0.0001);
        assertEquals(42, (Integer) intVal);
        assertEquals("hello", strVal);
        assertEquals(true, boolVal);
    }

    @Test
    public void testMultipleSaveAndLoad() {
        ConfigManager manager = ConfigManager.getTestInstance(tempDir);

        for (int i = 0; i < 5; i++) {
            manager.set("val" + i, i * 1.0);
            manager.saveNewVersion();
        }

        int[] versions = manager.getVersions();
        assertEquals(6, versions.length);
        assertArrayEquals(new int[] {1, 2, 3, 4, 5, 6}, versions);

        manager.loadVersion(4);
        assertEquals(2.0, (Double) manager.get("val2"), 0.0001);
    }

    @Test
    public void testSymlinkConsistency() throws IOException {
        ConfigManager manager = ConfigManager.getTestInstance(tempDir);
        manager.set("alpha", 1.23);
        manager.saveNewVersion(); // v2
        manager.set("beta", 4.56);
        manager.saveNewVersion(); // v3

        Path currentLink = tempDir.resolve("current.json");
        assertTrue(Files.isSymbolicLink(currentLink));

        Path target = Files.readSymbolicLink(currentLink);
        assertEquals("v3.json", target.toString());

        manager.loadVersion(2);
        Path target2 = Files.readSymbolicLink(currentLink);
        assertEquals("v2.json", target2.toString());

        assertEquals(1.23, (Double) manager.get("alpha"), 0.0001);
    }

    @Test
    public void testSingletonBehavior() {
        ConfigManager manager1 = ConfigManager.getTestInstance(tempDir);
        ConfigManager manager2 = ConfigManager.getTestInstance(tempDir);

        assertSame(manager1, manager2);

        manager1.set("x", 123);
        assertEquals(123, manager2.get("x"));
    }
}
