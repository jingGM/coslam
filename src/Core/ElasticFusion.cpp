//
// Created by jing on 7/12/21.
//

#include "ElasticFusion.h"

ElasticFusion::ElasticFusion(const int timeDelta, const int countThresh, const float errThresh, const float covThresh,
                             const bool closeLoops, const bool iclnuim, const float photoThresh, const float confidence,
                             const float depthCut, const float icpThresh, const bool fastOdom, const bool reloc,
                             const float fernThresh, const bool so3, const bool frameToFrameRGB,
                             const std::string fileName, const bool useGlobalCam)
:   frameToModel(LocalCameraInfo::getInstance().width(),
                 LocalCameraInfo::getInstance().height(),
                 LocalCameraInfo::getInstance().cx(),
                 LocalCameraInfo::getInstance().cy(),
                 LocalCameraInfo::getInstance().fx(),
                 LocalCameraInfo::getInstance().fy()),
    modelToModel(LocalCameraInfo::getInstance().width(),
                 LocalCameraInfo::getInstance().height(),
                 LocalCameraInfo::getInstance().cx(),
                 LocalCameraInfo::getInstance().cy(),
                 LocalCameraInfo::getInstance().fx(),
                 LocalCameraInfo::getInstance().fy()),
    tick(1),
    timeDelta(timeDelta),
    icpCountThresh(countThresh),
    icpErrThresh(errThresh),
    covThresh(covThresh),
    fernThresh(fernThresh),
    closeLoops(closeLoops),
    iclnuim(iclnuim),
    pyramid(true),
    fastOdom(fastOdom),
    reloc(reloc),
    lost(false),
    lastFrameRecovery(false),
    trackingCount(0),
    rgbOnly(false),
    useGlobalCam(useGlobalCam),
    icpWeight(icpThresh),
    so3(so3),
    depthCutoff(depthCut),
    maxDepthProcessed(20.0f),
    denseThreshold(0.75f),
    confidenceThreshold(confidence),
    deforms(0),
    fernDeforms(0),
    frameToFrameRGB(frameToFrameRGB),
    consSample(20),
    saveFilename(fileName),
    currPose(Eigen::Matrix4f::Identity()),
    resize(LocalCameraInfo::getInstance().width(),
           LocalCameraInfo::getInstance().height(),
           LocalCameraInfo::getInstance().width() / consSample,
           LocalCameraInfo::getInstance().height() / consSample),
    imageBuff(LocalCameraInfo::getInstance().rows() / consSample, LocalCameraInfo::getInstance().cols() / consSample),
    consBuff(LocalCameraInfo::getInstance().rows() / consSample, LocalCameraInfo::getInstance().cols() / consSample),
    timesBuff(LocalCameraInfo::getInstance().rows() / consSample, LocalCameraInfo::getInstance().cols() / consSample),
    ferns(500, depthCut * 1000, photoThresh)
{
    createTextures();
    createCompute();
    createFeedbackBuffers();

    std::string filename = fileName;
    filename.append(".freiburg");

    std::ofstream file;
    file.open(filename.c_str(), std::fstream::out);
    file.close();

    Stopwatch::getInstance().setCustomSignature(12431231);
}

ElasticFusion::~ElasticFusion()
{
    if(iclnuim)
    {
        savePly();
    }

    //Output deformed pose graph
    std::string fname = saveFilename;
    fname.append(".freiburg");

    std::ofstream f;
    f.open(fname.c_str(), std::fstream::out);

    for(size_t i = 0; i < poseGraph.size(); i++)
    {
        std::stringstream strs;

        if(iclnuim)
        {
            strs << std::setprecision(6) << std::fixed << (double)poseLogTimes.at(i) << " ";
        }
        else
        {
            strs << std::setprecision(6) << std::fixed << (double)poseLogTimes.at(i) / 1000000.0 << " ";
        }

        Eigen::Vector3f trans = poseGraph.at(i).second.topRightCorner(3, 1);
        Eigen::Matrix3f rot = poseGraph.at(i).second.topLeftCorner(3, 3);

        f << strs.str() << trans(0) << " " << trans(1) << " " << trans(2) << " ";

        Eigen::Quaternionf currentCameraRotation(rot);

        f << currentCameraRotation.x() << " " << currentCameraRotation.y() << " " << currentCameraRotation.z() << " " << currentCameraRotation.w() << "\n";
    }

    f.close();

    for(std::map<std::string, GPUTexture*>::iterator it = textures.begin(); it != textures.end(); ++it)
    {
        delete it->second;
    }

    textures.clear();

    for(std::map<std::string, ComputePack*>::iterator it = computePacks.begin(); it != computePacks.end(); ++it)
    {
        delete it->second;
    }

    computePacks.clear();

    for(std::map<std::string, FeedbackBuffer*>::iterator it = feedbackBuffers.begin(); it != feedbackBuffers.end(); ++it)
    {
        delete it->second;
    }

    feedbackBuffers.clear();
}

const int & ElasticFusion::getTick() const { return tick; }

void ElasticFusion::setTick(const int & val) { tick = val; }

void ElasticFusion::setPyramid(const bool & val) { pyramid = val; }

void ElasticFusion::setFastOdom(const bool & val) { fastOdom = val; }

void ElasticFusion::setRgbOnly(const bool & val) {rgbOnly = val;}

void ElasticFusion::setIcpWeight(const float & val) {icpWeight = val;}

void ElasticFusion::setSo3(const bool & val) {so3 = val;}

void ElasticFusion::setFrameToFrameRGB(const bool & val) {frameToFrameRGB = val;}

void ElasticFusion::setFernThresh(const float & val) {fernThresh = val;}

const bool & ElasticFusion::getLost() {return lost;}

const Eigen::Matrix4f & ElasticFusion::getCurrPose() {return currPose;}

std::map<std::string, GPUTexture*> & ElasticFusion::getTextures() {return textures;}

const float & ElasticFusion::getMaxDepthProcessed() {return maxDepthProcessed;}

std::map<std::string, FeedbackBuffer*> & ElasticFusion::getFeedbackBuffers() {return feedbackBuffers;}

GlobalModel & ElasticFusion::getGlobalModel() {return globalModel;}

const int & ElasticFusion::getTimeDelta() {return timeDelta;}

IndexMap & ElasticFusion::getIndexMap() {return indexMap;}

Ferns & ElasticFusion::getFerns() {return ferns;}

Deformation & ElasticFusion::getLocalDeformation() {return localDeformation;}

const int & ElasticFusion::getDeforms() {return deforms;}

const int & ElasticFusion::getFernDeforms() {return fernDeforms;}

void ElasticFusion::setDepthCutoff(const float & val) {depthCutoff = val;}

const float & ElasticFusion::getConfidenceThreshold() {return confidenceThreshold;}

void ElasticFusion::setConfidenceThreshold(const float & val) {confidenceThreshold = val;}

const RGBDOdometry & ElasticFusion::getModelToModel() {return modelToModel;}

const std::vector<PoseMatch> & ElasticFusion::getPoseMatches() {return poseMatches;}

void ElasticFusion::normaliseDepth(const float & minVal, const float & maxVal)
{
    std::vector<Uniform> uniforms;

    uniforms.push_back(Uniform("maxVal", maxVal * 1000.f));
    uniforms.push_back(Uniform("minVal", minVal * 1000.f));

    computePacks[ComputePack::NORM]->compute(textures[GPUTexture::DEPTH_RAW]->texture, &uniforms);
}

void ElasticFusion::savePly()
{
    std::string filename = saveFilename;
    filename.append(".ply");

    // Open file
    std::ofstream fs;
    fs.open (filename.c_str ());

    Eigen::Vector4f * mapData = globalModel.downloadMap();

    int validCount = 0;

    for(unsigned int i = 0; i < globalModel.lastCount(); i++)
    {
        Eigen::Vector4f pos = mapData[(i * 3) + 0];

        if(pos[3] > confidenceThreshold)
        {
            validCount++;
        }
    }

    // Write header
    fs << "ply";
    fs << "\nformat " << "binary_little_endian" << " 1.0";

    // Vertices
    fs << "\nelement vertex "<< validCount;
    fs << "\nproperty float x"
          "\nproperty float y"
          "\nproperty float z";

    fs << "\nproperty uchar red"
          "\nproperty uchar green"
          "\nproperty uchar blue";

    fs << "\nproperty float nx"
          "\nproperty float ny"
          "\nproperty float nz";

    fs << "\nproperty float radius";

    fs << "\nend_header\n";

    // Close the file
    fs.close ();

    // Open file in binary appendable
    std::ofstream fpout (filename.c_str (), std::ios::app | std::ios::binary);

    for(unsigned int i = 0; i < globalModel.lastCount(); i++)
    {
        Eigen::Vector4f pos = mapData[(i * 3) + 0];

        if(pos[3] > confidenceThreshold)
        {
            Eigen::Vector4f col = mapData[(i * 3) + 1];
            Eigen::Vector4f nor = mapData[(i * 3) + 2];

            nor[0] *= -1;
            nor[1] *= -1;
            nor[2] *= -1;

            float value;
            memcpy (&value, &pos[0], sizeof (float));
            fpout.write (reinterpret_cast<const char*> (&value), sizeof (float));

            memcpy (&value, &pos[1], sizeof (float));
            fpout.write (reinterpret_cast<const char*> (&value), sizeof (float));

            memcpy (&value, &pos[2], sizeof (float));
            fpout.write (reinterpret_cast<const char*> (&value), sizeof (float));

            unsigned char r = int(col[0]) >> 16 & 0xFF;
            unsigned char g = int(col[0]) >> 8 & 0xFF;
            unsigned char b = int(col[0]) & 0xFF;

            fpout.write (reinterpret_cast<const char*> (&r), sizeof (unsigned char));
            fpout.write (reinterpret_cast<const char*> (&g), sizeof (unsigned char));
            fpout.write (reinterpret_cast<const char*> (&b), sizeof (unsigned char));

            memcpy (&value, &nor[0], sizeof (float));
            fpout.write (reinterpret_cast<const char*> (&value), sizeof (float));

            memcpy (&value, &nor[1], sizeof (float));
            fpout.write (reinterpret_cast<const char*> (&value), sizeof (float));

            memcpy (&value, &nor[2], sizeof (float));
            fpout.write (reinterpret_cast<const char*> (&value), sizeof (float));

            memcpy (&value, &nor[3], sizeof (float));
            fpout.write (reinterpret_cast<const char*> (&value), sizeof (float));
        }
    }

    // Close file
    fs.close ();

    delete [] mapData;
}

void ElasticFusion::createTextures()
{
    textures[GPUTexture::RGB] = new GPUTexture(LocalCameraInfo::getInstance().width(),
                                               LocalCameraInfo::getInstance().height(),
                                               GL_RGBA,
                                               GL_RGB,
                                               GL_UNSIGNED_BYTE,
                                               true,
                                               true);

    textures[GPUTexture::DEPTH_RAW] = new GPUTexture(LocalCameraInfo::getInstance().width(),
                                                     LocalCameraInfo::getInstance().height(),
                                                     GL_LUMINANCE16UI_EXT,
                                                     GL_LUMINANCE_INTEGER_EXT,
                                                     GL_UNSIGNED_SHORT);

    textures[GPUTexture::DEPTH_FILTERED] = new GPUTexture(LocalCameraInfo::getInstance().width(),
                                                          LocalCameraInfo::getInstance().height(),
                                                          GL_LUMINANCE16UI_EXT,
                                                          GL_LUMINANCE_INTEGER_EXT,
                                                          GL_UNSIGNED_SHORT,
                                                          false,
                                                          true);

    textures[GPUTexture::DEPTH_METRIC] = new GPUTexture(LocalCameraInfo::getInstance().width(),
                                                        LocalCameraInfo::getInstance().height(),
                                                        GL_LUMINANCE32F_ARB,
                                                        GL_LUMINANCE,
                                                        GL_FLOAT);

    textures[GPUTexture::DEPTH_METRIC_FILTERED] = new GPUTexture(LocalCameraInfo::getInstance().width(),
                                                                 LocalCameraInfo::getInstance().height(),
                                                                 GL_LUMINANCE32F_ARB,
                                                                 GL_LUMINANCE,
                                                                 GL_FLOAT);

    textures[GPUTexture::DEPTH_NORM] = new GPUTexture(LocalCameraInfo::getInstance().width(),
                                                      LocalCameraInfo::getInstance().height(),
                                                      GL_LUMINANCE,
                                                      GL_LUMINANCE,
                                                      GL_FLOAT,
                                                      true);
    if(useGlobalCam) {
        textures[GPUTexture::GLOBAL_RAW] = new GPUTexture(GlobalCamInfo::getInstance().width(),
                                                          GlobalCamInfo::getInstance().height(),
                                                          GL_RGBA,
                                                          GL_RGB,
                                                          GL_UNSIGNED_BYTE,
                                                          true,
                                                          true);
        textures[GPUTexture::GLOBAL_FILTERED] = new GPUTexture(GlobalCamInfo::getInstance().width(),
                                                               GlobalCamInfo::getInstance().height(),
                                                               GL_RGBA,
                                                               GL_RGB,
                                                               GL_UNSIGNED_BYTE,
                                                               true,
                                                               true);
    }
}

void ElasticFusion::createCompute()
{
    // normalize the depth
    computePacks[ComputePack::NORM] = new ComputePack(loadProgramFromFile("empty.vert", "depth_norm.frag", "quad.geom"),
                                                      textures[GPUTexture::DEPTH_NORM]->texture);

    //smooth: compute the average value of the near by +- 3 pixels as the color
    computePacks[ComputePack::FILTER] = new ComputePack(loadProgramFromFile("empty.vert", "depth_bilateral.frag", "quad.geom"),
                                                        textures[GPUTexture::DEPTH_FILTERED]->texture);

    // threshold the data to the value [300u - maxD]
    computePacks[ComputePack::METRIC] = new ComputePack(loadProgramFromFile("empty.vert", "depth_metric.frag", "quad.geom"),
                                                        textures[GPUTexture::DEPTH_METRIC]->texture);

    computePacks[ComputePack::METRIC_FILTERED] = new ComputePack(loadProgramFromFile("empty.vert", "depth_metric.frag", "quad.geom"),
                                                                 textures[GPUTexture::DEPTH_METRIC_FILTERED]->texture);
}

void ElasticFusion::filterDepth()
{
    // set the target format
    std::vector<Uniform> uniforms;
    uniforms.emplace_back("cols", (float)LocalCameraInfo::getInstance().cols());
    uniforms.emplace_back("rows", (float)LocalCameraInfo::getInstance().rows());
    uniforms.emplace_back("maxD", depthCutoff);

    computePacks[ComputePack::FILTER]->compute(textures[GPUTexture::DEPTH_RAW]->texture, &uniforms);
}

void ElasticFusion::metriciseDepth()
{
    std::vector<Uniform> uniforms;

    uniforms.emplace_back("maxD", depthCutoff);

    computePacks[ComputePack::METRIC]->compute(textures[GPUTexture::DEPTH_RAW]->texture, &uniforms);
    computePacks[ComputePack::METRIC_FILTERED]->compute(textures[GPUTexture::DEPTH_FILTERED]->texture, &uniforms);
}

void ElasticFusion::createFeedbackBuffers()
{
    // after computepack, input is the texture coordinates
    // output:  position = [current frame position, confidence];
    //          Color = [[r,g,b,0], 0, time];
    //          vNormRad = [norm vector, rad=sqrt(2)*z*f/abs(nz)]
    feedbackBuffers[FeedbackBuffer::RAW] = new FeedbackBuffer(loadProgramGeomFromFile("vertex_feedback.vert", "vertex_feedback.geom"));
    feedbackBuffers[FeedbackBuffer::FILTERED] = new FeedbackBuffer(loadProgramGeomFromFile("vertex_feedback.vert", "vertex_feedback.geom"));
}

void ElasticFusion::computeFeedbackBuffers()
{
    // compute the surfel vertices and store it in each vbo buffer

    TICK("feedbackBuffers");
    feedbackBuffers[FeedbackBuffer::RAW]->compute(textures[GPUTexture::RGB]->texture,
                                                  textures[GPUTexture::DEPTH_METRIC]->texture,
                                                  tick,
                                                  maxDepthProcessed);

    feedbackBuffers[FeedbackBuffer::FILTERED]->compute(textures[GPUTexture::RGB]->texture,
                                                       textures[GPUTexture::DEPTH_METRIC_FILTERED]->texture,
                                                       tick,
                                                       maxDepthProcessed);
    TOCK("feedbackBuffers");
}

bool ElasticFusion::denseEnough(const Img<Eigen::Matrix<unsigned char, 3, 1>> & img)
{
    int sum = 0;

    for(int i = 0; i < img.rows; i++)
    {
        for(int j = 0; j < img.cols; j++)
        {
            sum += img.at<Eigen::Matrix<unsigned char, 3, 1>>(i, j)(0) > 0 &&
                   img.at<Eigen::Matrix<unsigned char, 3, 1>>(i, j)(1) > 0 &&
                   img.at<Eigen::Matrix<unsigned char, 3, 1>>(i, j)(2) > 0;
        }
    }

    return float(sum) / float(img.rows * img.cols) > denseThreshold;
}

Eigen::Vector3f ElasticFusion::rodrigues2(const Eigen::Matrix3f& matrix)
{
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(matrix, Eigen::ComputeFullV | Eigen::ComputeFullU);
    Eigen::Matrix3f R = svd.matrixU() * svd.matrixV().transpose();

    double rx = R(2, 1) - R(1, 2);
    double ry = R(0, 2) - R(2, 0);
    double rz = R(1, 0) - R(0, 1);

    double s = sqrt((rx*rx + ry*ry + rz*rz)*0.25);
    double c = (R.trace() - 1) * 0.5;
    c = c > 1. ? 1. : c < -1. ? -1. : c;

    double theta = acos(c);

    if( s < 1e-5 )
    {
        double t;

        if( c > 0 )
            rx = ry = rz = 0;
        else
        {
            t = (R(0, 0) + 1)*0.5;
            rx = sqrt( std::max(t, 0.0) );
            t = (R(1, 1) + 1)*0.5;
            ry = sqrt( std::max(t, 0.0) ) * (R(0, 1) < 0 ? -1.0 : 1.0);
            t = (R(2, 2) + 1)*0.5;
            rz = sqrt( std::max(t, 0.0) ) * (R(0, 2) < 0 ? -1.0 : 1.0);

            if( fabs(rx) < fabs(ry) && fabs(rx) < fabs(rz) && (R(1, 2) > 0) != (ry*rz > 0) )
                rz = -rz;
            theta /= sqrt(rx*rx + ry*ry + rz*rz);
            rx *= theta;
            ry *= theta;
            rz *= theta;
        }
    }
    else
    {
        double vth = 1/(2*s);
        vth *= theta;
        rx *= vth; ry *= vth; rz *= vth;
    }
    return Eigen::Vector3d(rx, ry, rz).cast<float>();
}

void ElasticFusion::processFerns()
{
    TICK("Ferns::addFrame");
    // add a frame which is not similar as previous frame
    // in 500 random valued thresholds, the max similarity of the frame with all previous frames are smaller than 1-fernThresh
    // The similarity is the value similarity of goodCodes of each frame
    ferns.addFrame(&fillIn.imageTexture, &fillIn.vertexTexture, &fillIn.normalTexture, currPose, tick, fernThresh);
    TOCK("Ferns::addFrame");
}

void ElasticFusion::predict()
{
    TICK("IndexMap::ACTIVE");

    if(lastFrameRecovery)
    {
        // give the corrected data to combinedFrameBuffer which contains:
        // imageTexture(decoded), vertexTexture(latest frame), normalTexture(latest frame)
        indexMap.combinedPredict(currPose,
                                 globalModel.model(),
                                 maxDepthProcessed,
                                 confidenceThreshold,
                                 0,
                                 tick,
                                 timeDelta,
                                 IndexMap::ACTIVE);
    }
    else
    {
        indexMap.combinedPredict(currPose,
                                 globalModel.model(),
                                 maxDepthProcessed,
                                 confidenceThreshold,
                                 tick,
                                 tick,
                                 timeDelta,
                                 IndexMap::ACTIVE);
    }

    TICK("FillIn");
    // if the indexMap vertex.z is 0, give the depth to fillIn.vertexTexture;  else give indexMap.vertexTex
    fillIn.vertex(indexMap.vertexTex(), textures[GPUTexture::DEPTH_FILTERED], lost);
    // if the indexMap normalTex.z is 0, give the depth normal to fillIn.normalTexture;  else give indexMap.normalTex
    fillIn.normal(indexMap.normalTex(), textures[GPUTexture::DEPTH_FILTERED], lost);
    // if the indexMap imageTex.r+g+b is 0, give the RGB to fillIn.imageTexture;  else give indexMap.imageTex
    fillIn.image(indexMap.imageTex(), textures[GPUTexture::RGB], lost || frameToFrameRGB);
    TOCK("FillIn");

    TOCK("IndexMap::ACTIVE");
}

void ElasticFusion::processFrame(const unsigned char * rgb,
                                 const unsigned short * depth,
                                 const int64_t & timestamp,
                                 const Eigen::Matrix4f * inPose,
                                 const float weightMultiplier,
                                 const bool bootstrap,
                                 const unsigned char * globalRGB)
{
    TICK("Run");

    textures[GPUTexture::DEPTH_RAW]->texture->Upload(depth, GL_LUMINANCE_INTEGER_EXT, GL_UNSIGNED_SHORT);
    textures[GPUTexture::RGB]->texture->Upload(rgb, GL_RGB, GL_UNSIGNED_BYTE);
    if(useGlobalCam)
    {
        textures[GPUTexture::GLOBAL_RAW]->texture->Upload(globalRGB, GL_RGB, GL_UNSIGNED_BYTE);
        textures[GPUTexture::GLOBAL_FILTERED]->texture->Upload(globalRGB, GL_RGB, GL_UNSIGNED_BYTE);
    }

    TICK("Preprocess");

    filterDepth();
    metriciseDepth();

    TOCK("Preprocess");

    //First run
    if(tick == 1)
    {
        computeFeedbackBuffers();

        // globalModel defines the total vertices that gui can send, but all in local frames
        // combine the raw depth and color with filtered norm, then pass it to rawFeedback.vbo
        globalModel.initialise(*feedbackBuffers[FeedbackBuffer::RAW],
                               *feedbackBuffers[FeedbackBuffer::FILTERED]);

        // give covert rgb to intensity and pass it to lastNextImage[0]
        // update lastNextImage[1,2] by shrink size of 2 and using Guassian kernel to compute.
        frameToModel.initFirstRGB(textures[GPUTexture::RGB]);
    }
    else
    {
        Eigen::Matrix4f lastPose = currPose;

        bool trackingOk = true;

        // decide if to use inPose which is ground truth
        if(bootstrap || !inPose)
        {
            TICK("autoFill");
            resize.image(indexMap.imageTex(), imageBuff);

            bool shouldFillIn = !denseEnough(imageBuff);
            TOCK("autoFill");

            TICK("odomInit");

            //WARNING initICP* must be called before initRGB*
            // copy vertices/normals to vmaps_tmp/nmaps_tmp and convert to global vmaps_g_prev_/nmaps_g_prev_
            frameToModel.initICPModel(shouldFillIn ? &fillIn.vertexTexture : indexMap.vertexTex(),
                                      shouldFillIn ? &fillIn.normalTexture : indexMap.normalTex(),
                                      maxDepthProcessed, currPose);
            // copy depth(vmaps_tmp.z) to lastDepth, rgb.cudares.texture to lastImage and do pyramid
            frameToModel.initRGBModel((shouldFillIn || frameToFrameRGB) ? &fillIn.imageTexture : indexMap.imageTex());

            // copy  the textures[GPUTexture::DEPTH_FILTERED] to depth_tmp[0] and use Guassian prramid to make pyramids
            // convert the depth_tmp to vmaps_curr_(vertex: x,y,z current frame) and nmaps_curr_()
            frameToModel.initICP(textures[GPUTexture::DEPTH_FILTERED], maxDepthProcessed);
            // copy depth(vmaps_tmp.z) to nextDepth, textures[GPUTexture::RGB] to nextImage and do pyramid
            frameToModel.initRGB(textures[GPUTexture::RGB]);
            TOCK("odomInit");

            if(bootstrap)
            {
                assert(inPose);
                currPose = currPose * (*inPose);
            }

            Eigen::Vector3f trans = currPose.topRightCorner(3, 1);
            Eigen::Matrix<float, 3, 3, Eigen::RowMajor> rot = currPose.topLeftCorner(3, 3);

            TICK("odom");
            // optimize trans and rot, get the transformation in global frame
            // use the vertices of current 200 frames estimate the transformation of the new frame
            frameToModel.getIncrementalTransformation(trans,
                                                      rot,
                                                      rgbOnly,
                                                      icpWeight,
                                                      pyramid,
                                                      fastOdom,
                                                      so3);
            TOCK("odom");

            trackingOk = frameToModel.lastICPError < 1e-04;

            if(reloc)
            {
                if(!lost)
                {
                    Eigen::MatrixXd covariance = frameToModel.getCovariance();

                    for(int i = 0; i < 6; i++)
                    {
                        if(covariance(i, i) > 1e-04)
                        {
                            trackingOk = false;
                            break;
                        }
                    }

                    if(!trackingOk)
                    {
                        trackingCount++;

                        if(trackingCount > 10)
                        {
                            lost = true;
                        }
                    }
                    else
                    {
                        trackingCount = 0;
                    }
                }
                else if(lastFrameRecovery)
                {
                    Eigen::MatrixXd covariance = frameToModel.getCovariance();

                    for(int i = 0; i < 6; i++)
                    {
                        if(covariance(i, i) > 1e-04)
                        {
                            trackingOk = false;
                            break;
                        }
                    }

                    if(trackingOk)
                    {
                        lost = false;
                        trackingCount = 0;
                    }

                    lastFrameRecovery = false;
                }
            }

            currPose.topRightCorner(3, 1) = trans;
            currPose.topLeftCorner(3, 3) = rot;
        }
        else
        {
            currPose = *inPose;
        }

        Eigen::Matrix4f diff = currPose.inverse() * lastPose;

        Eigen::Vector3f diffTrans = diff.topRightCorner(3, 1);
        Eigen::Matrix3f diffRot = diff.topLeftCorner(3, 3);

        //Weight by velocity
        float weighting = std::max(diffTrans.norm(), rodrigues2(diffRot).norm());

        float largest = 0.01;
        float minWeight = 0.5;

        if(weighting > largest)
        {
            weighting = largest;
        }

        // depends on rotation axis with angle
        weighting = std::max(1.0f - (weighting / largest), minWeight) * weightMultiplier;

        std::vector<Ferns::SurfaceConstraint> constraints;

        predict();

        Eigen::Matrix4f recoveryPose = currPose;

        if(closeLoops)
        {
            lastFrameRecovery = false;

            TICK("Ferns::findFrame");


            // return estimated pose: identity(1st)
            // constraints includes all vert points with the 1st optimized point and the second optimized point with closed loop
            // give lastClosest to the most similar frame id
            recoveryPose = ferns.findFrame(constraints,
                                           currPose,
                                           &fillIn.vertexTexture,
                                           &fillIn.normalTexture,
                                           &fillIn.imageTexture,
                                           tick,
                                           lost);
            TOCK("Ferns::findFrame");
        }

        std::vector<float> rawGraph;

        bool fernAccepted = false;

        // TODO: need to understand when ferns.lastClosest != -1, also to understand the fernMatch and relaxGraph
        if(closeLoops && ferns.lastClosest != -1)
        {
            if(lost)
            {
                currPose = recoveryPose;
                lastFrameRecovery = true;
            }
            else
            {
                for(size_t i = 0; i < constraints.size(); i++)
                {
                    globalDeformation.addConstraint(constraints.at(i).sourcePoint,
                                                    constraints.at(i).targetPoint,
                                                    tick,
                                                    ferns.frames.at(ferns.lastClosest)->srcTime,
                                                    true);
                }

                // TODO: recheck when the relativeCons is not empty
                for(size_t i = 0; i < relativeCons.size(); i++)
                {
                    globalDeformation.addConstraint(relativeCons.at(i));
                }

                if(globalDeformation.constrain(ferns.frames, rawGraph, tick, true, poseGraph, true))
                {
                    currPose = recoveryPose;

                    poseMatches.push_back(PoseMatch(ferns.lastClosest,
                                                    ferns.frames.size(), ferns.frames.at(ferns.lastClosest)->pose, currPose, constraints, true));

                    fernDeforms += rawGraph.size() > 0;

                    fernAccepted = true;
                }
            }
        }

        //If we didn't match to a fern
        if(!lost && closeLoops && rawGraph.size() == 0)
        {
            // Only predict old view, since we just predicted the current view for the ferns (which failed!)
            // Give everything of the world frame to the old_buffer
            TICK("IndexMap::INACTIVE");
            indexMap.combinedPredict(currPose,
                                     globalModel.model(),
                                     maxDepthProcessed,
                                     confidenceThreshold,
                                     0,
                                     tick - timeDelta, // the time bigger than the nearest timeDelta time will be discarded
                                     timeDelta,
                                     IndexMap::INACTIVE);
            TOCK("IndexMap::INACTIVE");

            //WARNING initICP* must be called before initRGB*
            modelToModel.initICPModel(indexMap.oldVertexTex(), indexMap.oldNormalTex(), maxDepthProcessed, currPose);
            modelToModel.initRGBModel(indexMap.oldImageTex());

            modelToModel.initICP(indexMap.vertexTex(), indexMap.normalTex(), maxDepthProcessed);
            modelToModel.initRGB(indexMap.imageTex());

            Eigen::Vector3f trans = currPose.topRightCorner(3, 1);
            Eigen::Matrix<float, 3, 3, Eigen::RowMajor> rot = currPose.topLeftCorner(3, 3);

            // use vertices before 200 frames estimate the rotation and transformation of the current 200 frames
            modelToModel.getIncrementalTransformation(trans,
                                                      rot,
                                                      false,
                                                      10,
                                                      pyramid,
                                                      fastOdom,
                                                      false);

            Eigen::MatrixXd covar = modelToModel.getCovariance();
            bool covOk = true;

            auto t = tick;
            for(int i = 0; i < 6; i++)
            {
                if(covar(i, i) > covThresh)
                {
                    covOk = false;
                    break;
                }
            }

            Eigen::Matrix4f estPose = Eigen::Matrix4f::Identity();

            estPose.topRightCorner(3, 1) = trans;
            estPose.topLeftCorner(3, 3) = rot;

            if(covOk && modelToModel.lastICPCount > icpCountThresh && modelToModel.lastICPError < icpErrThresh)
            {

                resize.vertex(indexMap.vertexTex(), consBuff);
                resize.time(indexMap.oldTimeTex(), timesBuff);

                for(int i = 0; i < consBuff.cols; i++)
                {
                    for(int j = 0; j < consBuff.rows; j++)
                    {
                        if(consBuff.at<Eigen::Vector4f>(j, i)(2) > 0 &&
                           consBuff.at<Eigen::Vector4f>(j, i)(2) < maxDepthProcessed &&
                           timesBuff.at<unsigned short>(j, i) > 0)
                        {
                            Eigen::Vector4f worldRawPoint = currPose * Eigen::Vector4f(consBuff.at<Eigen::Vector4f>(j, i)(0),
                                                                                       consBuff.at<Eigen::Vector4f>(j, i)(1),
                                                                                       consBuff.at<Eigen::Vector4f>(j, i)(2),
                                                                                       1.0f);

                            Eigen::Vector4f worldModelPoint = estPose * Eigen::Vector4f(consBuff.at<Eigen::Vector4f>(j, i)(0),
                                                                                        consBuff.at<Eigen::Vector4f>(j, i)(1),
                                                                                        consBuff.at<Eigen::Vector4f>(j, i)(2),
                                                                                        1.0f);

                            constraints.push_back(Ferns::SurfaceConstraint(worldRawPoint, worldModelPoint));

                            localDeformation.addConstraint(worldRawPoint,
                                                           worldModelPoint,
                                                           tick,
                                                           timesBuff.at<unsigned short>(j, i),
                                                           deforms == 0);
                        }
                    }
                }

                std::vector<Deformation::Constraint> newRelativeCons;

                if(localDeformation.constrain(ferns.frames, rawGraph, tick, false, poseGraph, false, &newRelativeCons))
                {
                    poseMatches.push_back(PoseMatch(ferns.frames.size() - 1, ferns.frames.size(), estPose, currPose, constraints, false));

                    deforms += rawGraph.size() > 0;

                    currPose = estPose;

                    for(size_t i = 0; i < newRelativeCons.size(); i += newRelativeCons.size() / 3)
                    {
                        relativeCons.push_back(newRelativeCons.at(i));
                    }
                }
            }
          }

        if(!rgbOnly && trackingOk && !lost)
        {
            TICK("indexMap");
            // get vertices within 200frames to indexFrameBuffer
            indexMap.predictIndices(currPose, tick, globalModel.model(), maxDepthProcessed, timeDelta);
            TOCK("indexMap");

            globalModel.fuse(currPose,
                             tick,
                             textures[GPUTexture::RGB],
                             textures[GPUTexture::DEPTH_METRIC],
                             textures[GPUTexture::DEPTH_METRIC_FILTERED],
                             indexMap.indexTex(),
                             indexMap.vertConfTex(),
                             indexMap.colorTimeTex(),
                             indexMap.normalRadTex(),
                             maxDepthProcessed,
                             confidenceThreshold,
                             weighting);

            TICK("indexMap");
            indexMap.predictIndices(currPose, tick, globalModel.model(), maxDepthProcessed, timeDelta);
            TOCK("indexMap");

            //If we're deforming we need to predict the depth again to figure out which
            //points to update the timestamp's of, since a deformation means a second pose update
            //this loop
            // TODO: check when there is closed loop
            if(rawGraph.size() > 0 && !fernAccepted)
            {
                indexMap.synthesizeDepth(currPose,
                                         globalModel.model(),
                                         maxDepthProcessed,
                                         confidenceThreshold,
                                         tick,
                                         tick - timeDelta,
                                         std::numeric_limits<unsigned short>::max());
            }

            globalModel.clean(currPose,
                              tick,
                              indexMap.indexTex(),
                              indexMap.vertConfTex(),
                              indexMap.colorTimeTex(),
                              indexMap.normalRadTex(),
                              indexMap.depthTex(),
                              confidenceThreshold,
                              rawGraph,
                              timeDelta,
                              maxDepthProcessed,
                              fernAccepted);
        }
    }

    poseGraph.emplace_back(tick, currPose);
    poseLogTimes.push_back(timestamp);

    TICK("sampleGraph");

    // sample 1/5000 vertices, maximum is 1024 vertices
    // give all the poses and times to the graphNodes in the def
    localDeformation.sampleGraphModel(globalModel.model());

    // sample 1/5 vertices of the localDeformation.graphPosePoints
    // initialize def
    globalDeformation.sampleGraphFrom(localDeformation);

    TOCK("sampleGraph");

    predict();

   if(!lost)
   {
       processFerns();
       tick++;
   }

    TOCK("Run");
}