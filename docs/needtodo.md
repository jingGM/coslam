#### Elastic Fusion
> initialization:
> * use the global pose as initial pose




#### ablation study:
> need to compare this initialization with original method
> 



#### Questions:
1.RGBDOdometry.cpp: 334: lastNextImage nextImage

2. GPUConfig.h: why thosse values?:
>  icpStepThreads(128), icpStepBlocks(112),
   rgbStepThreads(128), rgbStepBlocks(112),
   rgbResThreads(256),  rgbResBlocks(336),
   so3StepThreads(160), so3StepBlocks(64) 
