/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.utils.tuning;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.Reader;
import java.io.Writer;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/** Store config data on the rio */
public class ConfigManager {
    private static ConfigManager INSTANCE = null;
    private final Gson gson = new GsonBuilder().setPrettyPrinting().create();

    private final Path configDir;
    private final Path currentLink;

    private Path currVerFile;
    private RobotConfig currConfig;

    private boolean ready = false;
    private final List<Runnable> readyListeners = new ArrayList<>();
    private final List<Tuneable> tuneables = new ArrayList<>();

    /**
     * Get the current instance of config manager or create it if it doesn't exist
     *
     * @return An instance of ConfigManager
     */
    public static synchronized ConfigManager getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new ConfigManager(
                    Path.of(Filesystem.getDeployDirectory().toPath().toString(), "../configs/"));
        }

        return INSTANCE;
    }

    /**
     * Get the current instance of config manager or create it if it doesn't exist
     *
     * @return An instance of ConfigManager
     */
    public static synchronized ConfigManager getTestInstance(Path configDir) {
        if (INSTANCE == null) {
            INSTANCE = new ConfigManager(configDir);
            INSTANCE.init();
        }

        return INSTANCE;
    }

    public void registerTunable(Tuneable t) {
        tuneables.add(t);
    }

    public synchronized void onReady(Runnable r) {
        if (ready) {
            r.run();
        } else {
            readyListeners.add(r);
        }
    }

    public void init() {
        if (ready) return;
        ready = true;

        // Init tunables 1st
        System.out.println(tuneables.size() + " Tuneable values");
        tuneables.forEach(Tuneable::initFromConfig);

        // Run ready listeners

        System.out.println(readyListeners);
        readyListeners.forEach(Runnable::run);
        readyListeners.clear();
    }

    /**
     * Get a value from the curret config
     *
     * @param key The key
     * @return The value
     */
    public Object get(String key) {
        return this.currConfig.values.get(key);
    }

    /**
     * Set a value in the config
     *
     * @param key
     * @param value
     */
    public void set(String key, Object value) {
        this.currConfig.values.put(key, value);
        this.saveCurrentVersion();
    }

    public boolean contains(String key) {
        return this.currConfig.values.containsKey(key);
    }

    public void debug() {
        System.out.println(this.currConfig.toString());
        System.out.println(this.currVerFile.toString());
        System.out.println(this.currentLink.toString());

        try (BufferedReader reader = Files.newBufferedReader(this.currVerFile)) {
            while (true) {
                String line = reader.readLine();
                if (line == null) break;
                System.out.println(line);
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /** Initalize config manager and load the current config */
    private ConfigManager(Path configDir) {
        this.configDir = configDir;
        this.currentLink = configDir.resolve("current.json");
        try {
            Files.createDirectories(this.configDir);

            if (Files.isSymbolicLink(currentLink)) {
                this.currVerFile = Files.readSymbolicLink(currentLink);
                if (!this.currVerFile.isAbsolute()) {
                    this.currVerFile = configDir.resolve(this.currVerFile);
                }
                this.loadFromFile(this.currVerFile);
            } else {
                this.currConfig = new RobotConfig();
                currConfig.version = 1;
                this.saveNewVersion();
            }
        } catch (IOException e) {
            throw new RuntimeException("Failed to initialize config manager", e);
        }
    }

    /**
     * Save a version of the config file
     *
     * @param version The version to save
     */
    public void saveVersion(int version) {
        try {
            // System.out.println("saving version " + version);
            this.currConfig.version = version;
            Path versionFile = configDir.resolve("v" + version + ".json");

            Writer writer = Files.newBufferedWriter(versionFile);
            gson.toJson(this.currConfig, writer);
            writer.flush();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void saveCurrentVersion() {
        this.saveVersion(this.currConfig.version);
    }

    /**
     * Save a new version This will save the config to a new file and also update the current file
     * symlink
     */
    public void saveNewVersion() {
        int version = getNextVersionNumber();
        this.saveVersion(version);
        try {
            Path versionFile = configDir.resolve("v" + version + ".json");
            if (Files.exists(currentLink)) {
                Files.delete(currentLink);
            }

            Files.createSymbolicLink(currentLink, versionFile.getFileName());
            currVerFile = versionFile;
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Load an exsiting version
     *
     * @param version The version number to load
     */
    public void loadVersion(int version) {
        try {
            Path versionFile = configDir.resolve("v" + version + ".json");
            if (!Files.exists(versionFile)) {
                System.err.println("Version " + version + " does not exist.");
                return;
            }

            loadFromFile(versionFile);

            if (Files.exists(currentLink)) {
                Files.delete(currentLink);
            }
            Files.createSymbolicLink(currentLink, versionFile.getFileName());
            this.currVerFile = versionFile;
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public int[] getVersions() {
        try (var files = Files.list(configDir)) {
            return files.map(Path::getFileName)
                    .map(Path::toString)
                    .filter(name -> name.matches("v\\d+\\.json"))
                    .mapToInt(name -> Integer.parseInt(name.replaceAll("\\D", "")))
                    .sorted()
                    .toArray();
        } catch (IOException e) {
            e.printStackTrace();
            return new int[0];
        }
    }

    /**
     * Load config from a file
     *
     * @param file The path to load from
     */
    private void loadFromFile(Path file) {
        try (Reader reader = Files.newBufferedReader(file)) {
            this.currConfig = gson.fromJson(reader, RobotConfig.class);
            // System.out.println(this.currConfig.toString());
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Get the next version numbers based of the already existing files
     *
     * @return The version number
     */
    private int getNextVersionNumber() {
        try (var files = Files.list(configDir)) {
            return files.map(Path::getFileName)
                            .map(Path::toString)
                            .filter(name -> name.matches("v\\d+\\.json"))
                            .mapToInt(name -> Integer.parseInt(name.replaceAll("\\D", "")))
                            .max()
                            .orElse(0)
                    + 1;
        } catch (IOException e) {
            return 1;
        }
    }

    /** A wrapper class to store config versions */
    public class RobotConfig {
        public int version;
        public HashMap<String, Object> values = new HashMap<>();

        @Override
        public String toString() {
            return String.format("RobotConfig{version=%s,config=%s}", version, values.toString());
        }
    }
}
