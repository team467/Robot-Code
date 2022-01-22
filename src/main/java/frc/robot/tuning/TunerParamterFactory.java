package frc.robot.tuning;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;

public class TunerParamterFactory {
    public static TunerParameter create(String name, Tuner tuner, NetworkTableType type) {
        TunerParameter tunerParameter = new TunerParameter() {
            private String paramterName = name;
            private NetworkTableEntry parameterEntry = NetworkTableInstance.getDefault().getTable("tuning").getSubTable(tuner.getTunerName()).getEntry(name);
            private NetworkTableType parameterType = type;
            private ParameterValue defaultValue = new ParameterValue() {};

            @Override
            public String getName() {
                return paramterName;
            }

            @Override
            public NetworkTableEntry getEntry() {
                return parameterEntry;
            }

            @Override
            public NetworkTableType getType() {
                return parameterType;
            }

            @Override
            public void setDefaultValue(ParameterValue value) {
                this.defaultValue = value;
                switch (value.getType()) {
                    case kBoolean:
                        parameterEntry.setDefaultBoolean(value.getBoolean());
                        break;
                    case kBooleanArray:
                        parameterEntry.setDefaultBooleanArray(value.getBooleanArray());
                        break;
                    case kDouble:
                        parameterEntry.setDefaultDouble(value.getDouble());
                        break;
                    case kDoubleArray:
                        parameterEntry.setDefaultDoubleArray(value.getDoubleArray());
                        break;
                    case kRaw:
                        parameterEntry.setDefaultRaw(value.getRaw());
                        break;
                    case kString:
                        parameterEntry.setDefaultString(value.getString());
                        break;
                    case kStringArray:
                        parameterEntry.setDefaultStringArray(value.getStringArray());
                        break;
                    case kUnassigned:
                    case kRpc:
                    default:
                        break;
                }
            }

            @Override
            public void setValue(ParameterValue value) {
                switch (value.getType()) {
                    case kBoolean:
                        parameterEntry.setBoolean(value.getBoolean());
                        break;
                    case kBooleanArray:
                        parameterEntry.setBooleanArray(value.getBooleanArray());
                        break;
                    case kDouble:
                        parameterEntry.setDouble(value.getDouble());
                        break;
                    case kDoubleArray:
                        parameterEntry.setDoubleArray(value.getDoubleArray());
                        break;
                    case kRaw:
                        parameterEntry.setRaw(value.getRaw());
                        break;
                    case kString:
                        parameterEntry.setString(value.getString());
                        break;
                    case kStringArray:
                        parameterEntry.setStringArray(value.getStringArray());
                        break;
                    case kUnassigned:
                    case kRpc:
                    default:
                        break;
                }
            }

            @Override
            public ParameterValue getValue() {
                switch (parameterType) {
                    case kBoolean:
                        return new ParameterValue() {
                            @Override
                            public NetworkTableType getType() {
                                return NetworkTableType.kBoolean;
                            }

                            @Override
                            public boolean getBoolean() {
                                return parameterEntry.getBoolean(defaultValue.getBoolean());
                            }
                        };
                    case kBooleanArray:
                        return new ParameterValue() {
                            @Override
                            public NetworkTableType getType() {
                                return NetworkTableType.kBooleanArray;
                            }

                            @Override
                            public boolean[] getBooleanArray() {
                                return parameterEntry.getBooleanArray(defaultValue.getBooleanArray());
                            }
                        };
                    case kDouble:
                        return new ParameterValue() {
                            @Override
                            public NetworkTableType getType() {
                                return NetworkTableType.kDouble;
                            }

                            @Override
                            public double getDouble() {
                                return parameterEntry.getDouble(defaultValue.getDouble());
                            }
                        };
                    case kDoubleArray:
                        return new ParameterValue() {
                            @Override
                            public NetworkTableType getType() {
                                return NetworkTableType.kDoubleArray;
                            }

                            @Override
                            public double[] getDoubleArray() {
                                return parameterEntry.getDoubleArray(defaultValue.getDoubleArray());
                            }
                        };
                    case kRaw:
                        return new ParameterValue() {
                            @Override
                            public NetworkTableType getType() {
                                return NetworkTableType.kRaw;
                            }

                            @Override
                            public byte[] getRaw() {
                                return parameterEntry.getRaw(defaultValue.getRaw());
                            }
                        };
                    case kString:
                        return new ParameterValue() {
                            @Override
                            public NetworkTableType getType() {
                                return NetworkTableType.kString;
                            }

                            @Override
                            public String getString() {
                                return parameterEntry.getString(defaultValue.getString());
                            }
                        };
                    case kStringArray:
                        return new ParameterValue() {
                            @Override
                            public NetworkTableType getType() {
                                return NetworkTableType.kStringArray;
                            }

                            @Override
                            public String[] getStringArray() {
                                return parameterEntry.getStringArray(defaultValue.getStringArray());
                            }
                        };
                    case kUnassigned:
                    case kRpc:
                    default:
                        return new ParameterValue() {};
                }
            }

        };

        tunerParameter.setValue(new ParameterValue() {
            public NetworkTableType getType() {
                return type;
            };
        });
        return tunerParameter;
    }
}
