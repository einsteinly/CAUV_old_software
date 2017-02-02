## Contents

0. Notes
1. Directory Layout
2. Readmes
3. Messaging

## Notes

These readmes should provide a good layout of what is where. They should be used in tandem with entries on the wiki.

## Directory Layout

```
auv/ - Software designed to run on the vehicle
    control/
    img-pipeline/
    scripting/
common/ - Libraries for both vehicle and gui side
    pipeline_model/
utility/
gui/
```

## Readme

readme.txt - You are here
readme_cpp.txt - Information for all things C++

readme_outofdate - Old readmes that may still contain useful information

## Messaging

Message types:

```
    -cauv_control: stored in auv/control/msg
        -Attitude
        -ControlToken
        -DepthCalibration
        -ExternalMotorDemand
        -PIDParams
        -PIDState
        -PIDTarget
        -SimAttitude

    -cauv_pipeline_model: stored in common/pipeline_model/msg:
        -
```

Message Topics Diagram

```
    -control/
        -motors
        -attitude
        -depth
    -pipelines/
        -updates/
            -pipeline_1_name - Bool
            -...
        -new - String

```

Parameter Topics Diagram

```
    -imaging/
        -pipelines/
            -pipeline_1_name
            -...
        -processes/
            -process_1_name/
                -slot_1
                -...
            -...

```