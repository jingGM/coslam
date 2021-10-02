//
// Created by jing on 7/12/21.
//

#ifndef SLAM_ELASTICFUSION_H
#define SLAM_ELASTICFUSION_H
#include <memory>
#include <iomanip>
#include <Eigen/Dense>
#include "../GLDisplay/Resize.h"
#include "../GLDisplay/ComputePack.h"
#include "../GLDisplay/FeedbackBuffer.h"
#include "../GLDisplay/GlobalModel.h"
#include "../GLDisplay/FillIn.h"
#include "../GLDisplay/GlobalCamTexture.h"
#include "../Utils/LocalCameraInfo.h"
#include "../Utils/GlobalCamInfo.h"
#include "../Utils/Stopwatch.h"
#include "RGBDOdometry.h"
#include "GlobalOdometry.h"
#include "Ferns.h"
#include "Deformation.h"
#include "PoseMatch.h"

class ElasticFusion {
public:
    ElasticFusion(const int timeDelta = 200,
                  const int countThresh = 35000,
                  const float errThresh = 5e-05,
                  const float covThresh = 1e-05,
                  const bool closeLoops = true,
                  const bool iclnuim = false,
                  const float photoThresh = 115,
                  const float confidence = 10,
                  const float depthCut = 3,
                  const float icpThresh = 10,
                  const bool fastOdom = false,
                  const bool reloc = false,
                  const float fernThresh = 0.3095,
                  const bool so3 = true,
                  const bool frameToFrameRGB = false,
                  const std::string fileName = "",
                  const bool useGlobalCam = false,
                  const bool deformation = true);

    virtual ~ElasticFusion();

    /**
     * Predicts the current view of the scene, updates the [vertex/normal/image]Tex() members
     * of the indexMap class
     */
    void predict();

    /**
         * Process an rgb/depth map pair
         * @param rgb unsigned char row major order
         * @param depth unsigned short z-depth in millimeters, invalid depths are 0
         * @param timestamp nanoseconds (actually only used for the output poses, not important otherwise)
         * @param inPose optional input SE3 pose (if provided, we don't attempt to perform tracking)
         * @param weightMultiplier optional full frame fusion weight
         * @param bootstrap if true, use inPose as a pose guess rather than replacement
         */
    void processFrame(const unsigned char * rgb,
                      const unsigned short * depth,
                      const int64_t & timestamp,
                      const Eigen::Matrix4f * inPose = 0,
                      const float weightMultiplier = 1.f,
                      const bool bootstrap = false,
                      const unsigned char * globalRGB= nullptr);

    /**
         * Cheat the clock, only useful for multisession/log fast forwarding
         * @param val control time itself!
         */
    void setTick(const int & val);

    /**
         * Get the internal clock value of the fusion process
         * @return monotonically increasing integer value (not real-world time)
         */
    const int & getTick() const;

    /**
     * Internal maximum depth processed, this is defaulted to 20 (for rescaling depth buffers)
     * @return
     */
    const float & getMaxDepthProcessed();

    /**
     * This class contains the surfel map
     * @return
     */
    GlobalModel & getGlobalModel();

    /**
     * These are the vertex buffers computed from the raw input data
     * @return can be rendered
     */
    std::map<std::string, FeedbackBuffer*> & getFeedbackBuffers();

    /**
     * This class contains the local deformation graph
     * @return
     */
    Deformation & getLocalDeformation();

    /**
         * Whether or not to use a pyramid for tracking
         * @param val default is true
         */
    void setPyramid(const bool & val);

    /**
     * Controls the number of tracking iterations
     * @param val default is false
     */
    void setFastOdom(const bool & val);

    /**
     * If you set this to true we just do 2.5D RGB-only Lucas–Kanade tracking (no fusion)
     * @param val
     */
    void setRgbOnly(const bool & val);

    /**
     * Weight for ICP in tracking
     * @param val if 100, only use depth for tracking, if 0, only use RGB. Best value is 10
     */
    void setIcpWeight(const float & val);

    /**
     * Turns on or off SO(3) alignment bootstrapping
     * @param val
     */
    void setSo3(const bool & val);

    /**
     * Turns on or off frame to frame tracking for RGB
     * @param val
     */
    void setFrameToFrameRGB(const bool & val);

    /**
     * Returns whether or not the camera is lost, if relocalisation mode is on
     * @return
     */
    const bool & getLost();

    /**
         * The current global camera pose estimate
         * @return SE3 pose
         */
    const Eigen::Matrix4f & getCurrPose();

    /**
         * This is the map of raw input textures (you can display these)
         * @return
         */
    std::map<std::string, GPUTexture*> & getTextures();

    /**
     * Get the time window length for model matching
     * @return
     */
    const int & getTimeDelta();

    /**
     * This class contains all of the predicted renders
     * @return reference
     */
    IndexMap & getIndexMap();

    /**
     * This class contains the fern keyframe database
     * @return
     */
    Ferns & getFerns();

    /**
     * The number of local deformations that have occurred
     * @return
     */
    const int & getDeforms();

    /**
     * The number of global deformations that have occured
     * @return
     */
    const int & getFernDeforms();

    /**
     * This is the list of deformation constraints
     * @return
     */
    const std::vector<PoseMatch> & getPoseMatches();

    /**
     * Cut raw depth input off at this point
     * @param val default is 3 meters
     */
    void setDepthCutoff(const float & val);

    /**
     * Calculate the above for the current frame (only done on the first frame normally)
     */
    void computeFeedbackBuffers();

    /**
     * The point fusion confidence threshold
     * @return
     */
    const float & getConfidenceThreshold();

    /**
    * This is the tracking class, if you want access
    * @return
    */
    const RGBDOdometry & getModelToModel();

    /**
     * Raw data fusion confidence threshold
     * @param val default value is 10, but you can play around with this
     */
    void setConfidenceThreshold(const float & val);

    /**
     * Threshold for sampling new keyframes
     * @param val default is some magic value, change at your own risk
     */
    void setFernThresh(const float & val);

    /**
     * Saves out a .ply mesh file of the current model
     */
    void savePly();

    /**
     * Renders a normalised view of the input raw depth for displaying as an OpenGL texture
     * (this is stored under textures[GPUTexture::DEPTH_NORM]
     * @param minVal minimum depth value to render
     * @param maxVal maximum depth value to render
     */
    void normaliseDepth(const float & minVal, const float & maxVal);

private:
    void createTextures();
    void createCompute();
    void createFeedbackBuffers();

    void filterDepth();
    void metriciseDepth();

    bool denseEnough(const Img<Eigen::Matrix<unsigned char, 3, 1>> & img);

    void processFerns();

    Eigen::Vector3f rodrigues2(const Eigen::Matrix3f& matrix);

    GlobalModel globalModel;

    RGBDOdometry frameToModel;
    RGBDOdometry modelToModel;
    GlobalOdometry globalToModel;

    int tick;
    const int timeDelta;
    const int icpCountThresh;
    const float icpErrThresh;
    const float covThresh;
    float fernThresh;

    const bool closeLoops;
    const bool iclnuim;
    bool pyramid;
    bool fastOdom;
    const bool reloc;
    bool lost;
    bool lastFrameRecovery;
    int trackingCount;

    bool rgbOnly;
    bool useGlobalCam;
    float icpWeight;
    bool so3;

    float depthCutoff;
    const float maxDepthProcessed;
    float denseThreshold;
    float confidenceThreshold;

    int deforms;
    int fernDeforms;
    bool deformation;

    bool frameToFrameRGB;

    const int consSample;

    const std::string saveFilename;
    Eigen::Matrix4f currPose;
    Resize resize;

    Img<Eigen::Matrix<unsigned char, 3, 1>> imageBuff;
    Img<Eigen::Vector4f> consBuff;
    Img<unsigned short> timesBuff;

    std::vector<std::pair<unsigned long long int, Eigen::Matrix4f> > poseGraph;
    std::vector<unsigned long long int> poseLogTimes;

    std::vector<Deformation::Constraint> relativeCons;
    std::vector<PoseMatch> poseMatches;

    FillIn fillIn;
    IndexMap indexMap;
    GlobalCamTexture globalCamTex;

    Ferns ferns;

    std::map<std::string, GPUTexture*> textures;
    std::map<std::string, ComputePack*> computePacks;
    std::map<std::string, FeedbackBuffer*> feedbackBuffers;

    Deformation localDeformation;
    Deformation globalDeformation;

    std::vector<std::vector<double>> globalCamPose;
};

typedef std::shared_ptr<ElasticFusion> ElasticFusionPtr;

#endif //SLAM_ELASTICFUSION_H
