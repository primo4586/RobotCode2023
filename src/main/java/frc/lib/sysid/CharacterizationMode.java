package frc.lib.sysid;

public interface CharacterizationMode {
    double getVoltage(double time);

    String getTestMetaData();
}