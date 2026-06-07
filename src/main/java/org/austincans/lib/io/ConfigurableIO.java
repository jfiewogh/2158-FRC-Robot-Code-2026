package org.austincans.lib.io;

public interface ConfigurableIO<ConfigRequestsType> {
    public void applyConfigRequests(ConfigRequestsType configRequests);
}
