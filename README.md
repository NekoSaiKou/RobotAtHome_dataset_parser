# RobotAtHome_dataset_parser
Tools for reading Robot@Home Dataset

## File structures

* Root
    * Robot@Home-dataset_laser_scans-plain_text-all
        * 7 Sessions
            * Session
                * 6 Scenes
                    * {X_hokuyo_processed}
                    * {X_hokuyo_processed}.txt
    * Robot@Home-dataset_rgbd_data-plain_text-all
        * 7 Sessions
            * Session
                * {6 Scenes}
                * {6 Scenes}.txt

## How to use

1. Arrange the Robot@Home dataset structure as the format above
2. Build the executable using the provided script
3. Enjoy