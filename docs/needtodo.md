#### code to test:
The Interfaces in Tools folder

#### deleted from the original file
fillIn in rosElastic
resizeStream,streaming in rosElastic

#### need to delete
frameToSkip in rosElastic: put to logger files
icluim in ElasticFusion

#### Elastic Fusion
> Use the global pose 
>  * to optimize deformation


#### ablation study:
> need to compare this initialization with original method
> 



#### Questions:
1. blockReduceSum in reduce.cu: by RGBResidual
   * shared: size is not enough: one grid has many warps
   * val = (threadIdx.x < blockDim.x / warpSize) ? shared[lane] : zero;
   : only one block of warps?
   * change for loop to + only blockDim
   
2. ElasticFusion: 261,155: initRGBModel, initRGB all use vmaps_tmp.z to last and next image?

3. The splat.vert in IndexMap while processing the combinedPredict() function, normRad in indexMap shaders

4. globalDeformation.addConstraint, what is frame->srcTime
5. what is relativeCons

6. indexMap all points are stored in the frame and optimization only in the frame?

7. difference between render feedbackbuffer and glDrawTransformFeedback



#### today:
 - [ ] need to understand transform feedback: 134, 136, 143, 148
 - [ ] fix the Global Model file, add Index Map to the repo

 - [ ] finish understanding of the code
 - [ ] understand recoveryPose = ferns.findFrame



#### Think about
initialization of the chunk:
   1. get the average position or have the criterio for it
   2. keep the first frame as initialization

think about how to add odom to it.