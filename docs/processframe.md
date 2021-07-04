##ElasticFusion

#### Initializations
> 1. initialize indexMap: 
> > - create the current and historic opengl buffer for depth and rgb 
> > * program: index_map.vert .frag
> > * drawDepthProgram: visualise_textures.frag quad.geom
> > * depthProgram: splat.vert, depth_splat.frag
> > * combinedProgram: splat.vert, combo_splat.frag
> 2. FillIn:
> > - create image/vertices/normal opengl buffer
> > * imageProgram/v..Program/n..Program: fill_rgb.frag... quad.geom
> 3. resize:
> > * imageProgram, vertexProgram, timeProgram: resize.frag, quad.geom
> 4. textures: define GlTexture, register GLtextures to CUDA
> > * RGB, DEPTH_RAW, DEPTH_FILTERED, DEPTH_NORM: use cuda
> > * DEPTH_METRIC, DEPTH_METRIC_FILTERED: not use cuda
> 5. computePacks: define opengl/pangolin view
> > - use quad.geom
> > - .frag:
> >     * NORM: depth_norm.frag
> >     * FILTER: depth_bilateral
> >     * METRIC, METRIC_FILTERED: depth_metric
> 6. feedbackBuffers:
> > * give space to vetices(vbo) and UV, d define the NV feedback buffer
> > * draw GlSlProgram: draw_feedback.vert, draw_feedback.frag
> > * vertex GlSlProgram: RAW, FILTERED: vertex_feedback.vert, vertex_feedback.geom

####processFrame

>```
>textures[GPUTexture::DEPTH_RAW]->texture->Upload(depth, GL_LUMINANCE_INTEGER_EXT, GL_UNSIGNED_SHORT);
>textures[GPUTexture::RGB]->texture->Upload(rgb, GL_RGB, GL_UNSIGNED_BYTE);
>```
>give depth and rgb to textures DEPTH_RAW and RGB

> 1. `filterDepth()`
> > * `std::vector<Uniform> uniforms`: cols, rows, cutoff
> > * set uniform values to opengl: DEPTH_RAW computePack 
> > * compute the filtered texture
> > - [x] **need to know how**
> 2.  `metriciseDepth()`
> > * `std::vector<Uniform> uniforms`: cutoff
> > * set uniform values to opengl: METRIC, METRIC_FILTERED computePack
> > * compute the metric and metric_filtered texture
> > - [x] **need to know how**

> 3. First tick:
> > `computeFeedbackBuffers()`:
> > * RAW: use DEPTH_METRIC->texture
> > * FILTERED: use DEPTH_METRIC_FILTERED->texture
> > - [x] **need to know how if it is to draw the figure** 
> >
> > initialize globalModel by RAW and Filtered feedback buffer
> > - [x] **seems set feedback and render configurations** 
> >
> > initialize frameToModel by RGB texture
> > * give covert rgb to intensity and pass it to lastNextImage[0]
> > * update lastNextImage[1,2] by shrink size of 2 and using Guassian kernel to compute.

> 4. initial pose:
> > if there is initial pose: give the pose to currPose 
> 
> > if there is no intial pose:
> > * resize indexMap to size of imageBuff 20 times smaller 
> > - [x] **need to know how**
> > * test if there is enough pixels > 0
> > * frameToModel.initICPModel
> >     * copy predictedVertices/Normals to vmaps_g_prev_/nmaps_g_prev_[0](rows * 3, cols)
> >     * resize the maps for pyramid by 2:   vmaps_g_prev_
> >     - [x] **need to know how why src=dst 202 RGBDOdometry.cpp vmap_dst.create(rows * 3, cols);cudafuncs**
> >     * transform the vertices by currPose
> > * frameToModel.initRGBModel: input rgb
> >     * give vmaps_tmp's depth info to lastDepth and do Gaussian for pyramid
> >     * give rgb to lastImage and do Gaussian for pyramid
> > * frameToModel.initICP:
> >     * give textures[DEPTH_FILTERED] values to depth_tmp
> >     * depth_tmp[1,2]: pyramid by threshold of sigma_color and Gaussian-like average
> >     * convert real positions to vmaps_curr_: [x:(0-u,v),y:(u-2u,v),z:(2u-3u,v)]
> >     * create nmaps_curr by vmaps_curr: get x,y vectors and do cross product
> > * frameToModel.initRGB:
> >     * give vmaps_tmp's depth info to nextDepth and do Gaussian for pyramid
> >     * give rgb to nextImage and do Gaussian for pyramid

> 5. Odom transformation:
> > * Use sobel to compute the gradience of nextImage(rgb) to nextdIdx/nextIdy
> > * 


---

## RGBDOdometry:
> Sizes: texture: rgb+d
> > lastDepth/lastImage/lastNextImage/nextdIdx/nextdIdy/pointClouds/corresImg: 
> > * 3 pyramids
> > * 0:(width, height), 1:(width, height)/2, 2:(width, height)/4
> 
> > vmaps_g_prev_/nmaps_g_prev_(textures); vmaps_curr_/nmaps_curr_(actual)
> > * 3 pyramids: 0:(width, height), 1:(width, height)/2, 2:(width, height)/4
> > * each (pyr_rows*3, pyr_cols): x: (0-width, height), y:(width-2width, height), z:(2width-3width, height)
> 
> > vmaps_tmp/nmaps_tmp(textures);
> > (height * 4 * width): {[(0,1,2,3),(0,1,2,3)...],[...]}

> Sizes: matrix
> > temp:
> > * Rcurr = Rprev = rot; tcurr = tprev = trans
> > * R_lr = Identity; jtj:(3,3); jtr:(3,1)
> > * imageBasis = homography = K * resultR * kinv
> > * krlr = K * resultR