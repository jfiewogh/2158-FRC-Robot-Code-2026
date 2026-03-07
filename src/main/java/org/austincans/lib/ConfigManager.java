package org.austincans.lib;

import java.io.File;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import edu.wpi.first.wpilibj.Filesystem;

public class ConfigManager<T> {
    private static final ObjectMapper mapper = new ObjectMapper(new YAMLFactory());
    private final String filePath;
    private final Class<T> type;
    private T config;

    public ConfigManager(String filePath, Class<T> type) {
        this.filePath = filePath;
        this.type = type;
    }
    
    public void load() {
        File configFile = new File(Filesystem.getDeployDirectory(), filePath);
        try {
            config = mapper.readValue(configFile, type);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public T getConfig() {
        return config;
    }
}
