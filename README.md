# organize_velodyne_cloud
Repository for functions that are useful for organizing point clouds from a Velodyne LiDAR unit

How it works is this: every Velodyne LiDAR unit fires its lasers in a pre-defined sequence that repeats. E.g. 0, 16, 1, 17, 2, 18... But not every laser beam returns to the unit, so some parts of the cloud need to be filled in with points whose coordinates are `NaN` to preserve the organized nature of the cloud. My function creates a 2D organized cloud where there is one row for each "ring" of velodyne data. It then fills in each row, tracking how many points are in each row, i.e. what "column" of data is currently being filled in. When it moves from one column of data to the next, it fills in any gaps with `NaN` points.

Be careful how you use the organized cloud after this, as some PCL functions will crash if you input a cloud with `NaN`s, or worse, silently destroy the organized nature of the cloud without warning you.
