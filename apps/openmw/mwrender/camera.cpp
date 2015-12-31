#include <iostream>

#include "camera.hpp"

#include <osg/Camera>
#include <osg/View>
#include <osg/RenderInfo>

#include <components/sceneutil/positionattitudetransform.hpp>

#include "../mwbase/environment.hpp"
#include "../mwbase/windowmanager.hpp"

#include "../mwworld/ptr.hpp"
#include "../mwworld/refdata.hpp"

#include "npcanimation.hpp"

// NVidia GL_NVX_gpu_memory_info interface
// https://www.opengl.org/registry/specs/NVX/gpu_memory_info.txt
#define GPU_MEMORY_INFO_DEDICATED_VIDMEM_NVX          0x9047
#define GPU_MEMORY_INFO_TOTAL_AVAILABLE_MEMORY_NVX    0x9048
#define GPU_MEMORY_INFO_CURRENT_AVAILABLE_VIDMEM_NVX  0x9049
#define GPU_MEMORY_INFO_EVICTION_COUNT_NVX            0x904A
#define GPU_MEMORY_INFO_EVICTED_MEMORY_NVX            0x904B

// AMD GL_ATI_meminfo interface
// https://www.opengl.org/registry/specs/ATI/meminfo.txt
#define VBO_FREE_MEMORY_ATI                           0x87FB
#define TEXTURE_FREE_MEMORY_ATI                       0x87FC
#define RENDERBUFFER_FREE_MEMORY_ATI                  0x87FD

namespace
{

class UpdateRenderCameraCallback : public osg::NodeCallback
{
public:
    UpdateRenderCameraCallback(MWRender::Camera* cam)
        : mCamera(cam)
    {
    }

    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
    {
        osg::Camera* cam = static_cast<osg::Camera*>(node);

        // traverse first to update animations, in case the camera is attached to an animated node
        traverse(node, nv);

        mCamera->updateCamera(cam);
    }

private:
    MWRender::Camera* mCamera;
};

class MemoryInfoCallback : public osg::Camera::DrawCallback
{
private:
    enum ExtensionType {
        NONE,
        NVIDIA,
        AMD
    };

    // Which extension is being used?
    ExtensionType mExtension;

public:

    MemoryInfoCallback()
        : osg::Camera::DrawCallback()
        , mExtension(NONE)
    {
    }

    MemoryInfoCallback(osg::ref_ptr<osg::Camera> camera)
        : osg::Camera::DrawCallback()
        , mExtension(NONE)
    {
        // Initialize the memory statistics.
        osg::ref_ptr<osg::View> view = camera->getView();
        osg::Stats* stats = view->getStats();
        stats->setAttribute(0, "Texture memory", 0.0);

        osg::ref_ptr<osg::GraphicsContext> context = camera->getGraphicsContext();
        osg::ref_ptr<osg::State> state = context->getState();
        unsigned int contextID = state->getContextID();

        // Determine whether the required extensions are supported.
        if (osg::isGLExtensionSupported(contextID, "GL_NVX_gpu_memory_info")) {
            mExtension = NVIDIA;
        }
        else if (osg::isGLExtensionSupported(contextID, "GL_ATI_meminfo")) {
            mExtension = AMD;
            std::cout << "AMD GL_ATI_meminfo support is completely untested!" << std::endl;
        }
    }

    MemoryInfoCallback(const MemoryInfoCallback& copy, const osg::CopyOp& copyop)
        : osg::Camera::DrawCallback(copy, copyop)
        , mExtension(copy.mExtension)
    {
    }

    META_Object(OpenMW, MemoryInfoCallback)

    void operator()(osg::RenderInfo& renderInfo) const
    {
        if (mExtension == NONE) return;

        osg::ref_ptr<osg::View> view = renderInfo.getView();
        osg::Stats* stats = view->getStats();
        bool collecting = stats->collectStats("frame_rate");
        if (!collecting) return;

        osg::FrameStamp* frameStamp = view->getFrameStamp();
        int frameNumber = frameStamp->getFrameNumber();

        if (mExtension == NVIDIA) {
            // Dedicated and total appear to return the same numbers.
            GLint dedicated;
            glGetIntegerv(GPU_MEMORY_INFO_DEDICATED_VIDMEM_NVX, &dedicated);
            GLint total;
            glGetIntegerv(GPU_MEMORY_INFO_TOTAL_AVAILABLE_MEMORY_NVX, &total);
            GLint available;
            glGetIntegerv(GPU_MEMORY_INFO_CURRENT_AVAILABLE_VIDMEM_NVX, &available);
            GLint evictionCount;
            glGetIntegerv(GPU_MEMORY_INFO_EVICTION_COUNT_NVX, &evictionCount);
            GLint evictedMemory;
            glGetIntegerv(GPU_MEMORY_INFO_EVICTED_MEMORY_NVX, &evictedMemory);

            // Set the statistic in the view Stats object (requires modified OSG to display on screen).
            float mem_kb = available;
            stats->setAttribute(frameNumber, "Texture memory", mem_kb);

            // Occasional reporting for people without the modified version of OSG.
            if (frameNumber % 500 == 0) {
                std::cout << "Available video memory: dedicated=" << dedicated
                          << " total=" << total
                          << " available=" << available
                          << " ecount=" << evictionCount
                          << " ememory=" << evictedMemory << std::endl;
            }
        }
        else if (mExtension == AMD) {
            // The other two API parameters (VBO_FREE_MEMORY_ATI and RENDERBUFFER_FREE_MEMORY_ATI)
            // are reported to to return the same values as TEXTURE_FREE_MEMORY_ATI.  Presumably
            // modern cards don't differentiate in the same way that older cards did.
            GLint freeMem[4] = { 0, 0, 0, 0 };
            glGetIntegerv(TEXTURE_FREE_MEMORY_ATI, &freeMem[0]);

#if 0
            // Annoyingly, the total amount of video memory is available from a different extension,
            // that appears to be unsupported on Linux. :-(
            GLuint uNoOfGPUs = wglGetGPUIDsAMD(0, 0);
            GLuint* uGPUIDs = new GLuint[uNoOfGPUs];
            wglGetGPUIDsAMD(uNoOfGPUs, uGPUIDs);
            GLuint uTotalMemoryInMB = 0;
            wglGetGPUInfoAMD(uGPUIDs[0], WGL_GPU_RAM_AMD, GL_UNSIGNED_INT,
                             sizeof(GLuint), &uTotalMemoryInMB);
#endif

            // Set the statistic in the view Stats object (requires modified OSG to display on screen).
            float mem_kb = (float) freeMem[0];
            stats->setAttribute(frameNumber, "Texture memory", mem_kb);

            // Occasional reporting for people without the modified version of OSG.
            if (frameNumber % 500 == 0) {
                std::cout << "Available video memory: available=" << freeMem[0]
                          << " largest=" << freeMem[1]
                          << " Auxiliary (swap?) available=" << freeMem[2]
                          << " largest=" << freeMem[3] << std::endl;
            }
        }
    }
};

}

namespace MWRender
{

    Camera::Camera (osg::Camera* camera)
    : mHeightScale(1.f),
      mCamera(camera),
      mAnimation(NULL),
      mFirstPersonView(true),
      mPreviewMode(false),
      mFreeLook(true),
      mNearest(30.f),
      mFurthest(800.f),
      mIsNearest(false),
      mHeight(124.f),
      mMaxCameraDistance(192.f),
      mDistanceAdjusted(false),
      mVanityToggleQueued(false),
      mViewModeToggleQueued(false),
      mCameraDistance(0.f)
    {
        mVanity.enabled = false;
        mVanity.allowed = true;

        mPreviewCam.pitch = 0.f;
        mPreviewCam.yaw = 0.f;
        mPreviewCam.offset = 400.f;
        mMainCam.pitch = 0.f;
        mMainCam.yaw = 0.f;
        mMainCam.offset = 400.f;

        mUpdateCallback = new UpdateRenderCameraCallback(this);
        mCamera->addUpdateCallback(mUpdateCallback);

        // InitialDrawCallback is used for loading screens.
        // FinlDrawCallback is used for screenshots, but not on this camera.
        osg::ref_ptr<MemoryInfoCallback> memoryInfoCallback = new MemoryInfoCallback(mCamera);
        mCamera->setFinalDrawCallback(memoryInfoCallback);
    }

    Camera::~Camera()
    {
        mCamera->removeUpdateCallback(mUpdateCallback);
    }

    MWWorld::Ptr Camera::getTrackingPtr() const
    {
        return mTrackingPtr;
    }

    osg::Vec3d Camera::getFocalPoint()
    {
        const osg::Node* trackNode = mTrackingNode;
        if (!trackNode)
            return osg::Vec3d();
        osg::MatrixList mats = trackNode->getWorldMatrices();
        if (!mats.size())
            return osg::Vec3d();
        const osg::Matrix& worldMat = mats[0];

        osg::Vec3d position = worldMat.getTrans();
        if (!isFirstPerson())
            position.z() += mHeight * mHeightScale;
        return position;
    }

    void Camera::updateCamera(osg::Camera *cam)
    {
        if (mTrackingPtr.isEmpty())
            return;

        osg::Vec3d position = getFocalPoint();

        osg::Quat orient =  osg::Quat(getPitch(), osg::Vec3d(1,0,0)) * osg::Quat(getYaw(), osg::Vec3d(0,0,1));

        osg::Vec3d offset = orient * osg::Vec3d(0, -mCameraDistance, 0);
        position += offset;

        osg::Vec3d forward = orient * osg::Vec3d(0,1,0);
        osg::Vec3d up = orient * osg::Vec3d(0,0,1);

        cam->setViewMatrixAsLookAt(position, position + forward, up);
    }

    void Camera::reset()
    {
        togglePreviewMode(false);
        toggleVanityMode(false);
        if (!mFirstPersonView)
            toggleViewMode();
    }

    void Camera::rotateCamera(float pitch, float yaw, bool adjust)
    {
        if (adjust)
        {
            pitch += getPitch();
            yaw += getYaw();
        }
        setYaw(yaw);
        setPitch(pitch);
    }

    void Camera::attachTo(const MWWorld::Ptr &ptr)
    {
        mTrackingPtr = ptr;
    }

    void Camera::update(float duration, bool paused)
    {
        if (mAnimation->upperBodyReady())
        {
            // Now process the view changes we queued earlier
            if (mVanityToggleQueued)
            {
                toggleVanityMode(!mVanity.enabled);
                mVanityToggleQueued = false;
            }
            if (mViewModeToggleQueued)
            {

                togglePreviewMode(false);
                toggleViewMode();
                mViewModeToggleQueued = false;
            }
        }

        if (paused)
            return;

        // only show the crosshair in game mode and in first person mode.
        MWBase::WindowManager *wm = MWBase::Environment::get().getWindowManager();
        wm->showCrosshair(!wm->isGuiMode() && (mFirstPersonView && !mVanity.enabled && !mPreviewMode));

        if(mVanity.enabled)
        {
            rotateCamera(0.f, osg::DegreesToRadians(3.f * duration), true);
        }
    }

    void Camera::toggleViewMode(bool force)
    {
        // Changing the view will stop all playing animations, so if we are playing
        // anything important, queue the view change for later
        if (!mAnimation->upperBodyReady() && !force)
        {
            mViewModeToggleQueued = true;
            return;
        }
        else
            mViewModeToggleQueued = false;

        mFirstPersonView = !mFirstPersonView;
        processViewChange();

        if (mFirstPersonView) {
            mCameraDistance = 0.f;
        } else {
            mCameraDistance = mMaxCameraDistance;
        }
    }
    
    void Camera::allowVanityMode(bool allow)
    {
        if (!allow && mVanity.enabled)
            toggleVanityMode(false);
        mVanity.allowed = allow;
    }

    bool Camera::toggleVanityMode(bool enable)
    {
        // Changing the view will stop all playing animations, so if we are playing
        // anything important, queue the view change for later
        if (isFirstPerson() && !mAnimation->upperBodyReady())
        {
            mVanityToggleQueued = true;
            return false;
        }

        if(!mVanity.allowed && enable)
            return false;

        if(mVanity.enabled == enable)
            return true;
        mVanity.enabled = enable;

        processViewChange();

        float offset = mPreviewCam.offset;

        if (mVanity.enabled) {
            setPitch(osg::DegreesToRadians(-30.f));
            mMainCam.offset = mCameraDistance;
        } else {
            offset = mMainCam.offset;
        }

        mCameraDistance = offset;

        return true;
    }

    void Camera::togglePreviewMode(bool enable)
    {
        if (mFirstPersonView && !mAnimation->upperBodyReady())
            return;

        if(mPreviewMode == enable)
            return;

        mPreviewMode = enable;
        processViewChange();

        float offset = mCameraDistance;
        if (mPreviewMode) {
            mMainCam.offset = offset;
            offset = mPreviewCam.offset;
        } else {
            mPreviewCam.offset = offset;
            offset = mMainCam.offset;
        }

        mCameraDistance = offset;
    }

    void Camera::setSneakOffset(float offset)
    {
        mAnimation->setFirstPersonOffset(osg::Vec3f(0,0,-offset));
    }

    float Camera::getYaw()
    {
        if(mVanity.enabled || mPreviewMode)
            return mPreviewCam.yaw;
        return mMainCam.yaw;
    }

    void Camera::setYaw(float angle)
    {
        if (angle > osg::PI) {
            angle -= osg::PI*2;
        } else if (angle < -osg::PI) {
            angle += osg::PI*2;
        }
        if (mVanity.enabled || mPreviewMode) {
            mPreviewCam.yaw = angle;
        } else {
            mMainCam.yaw = angle;
        }
    }

    float Camera::getPitch()
    {
        if (mVanity.enabled || mPreviewMode) {
            return mPreviewCam.pitch;
        }
        return mMainCam.pitch;
    }

    void Camera::setPitch(float angle)
    {
        const float epsilon = 0.000001f;
        float limit = osg::PI_2 - epsilon;
        if(mPreviewMode)
            limit /= 2;

        if(angle > limit)
            angle = limit;
        else if(angle < -limit)
            angle = -limit;

        if (mVanity.enabled || mPreviewMode) {
            mPreviewCam.pitch = angle;
        } else {
            mMainCam.pitch = angle;
        }
    }

    float Camera::getCameraDistance() const
    {
        return mCameraDistance;
    }

    void Camera::setCameraDistance(float dist, bool adjust, bool override)
    {
        if(mFirstPersonView && !mPreviewMode && !mVanity.enabled)
            return;

        mIsNearest = false;

        if (adjust)
            dist += mCameraDistance;

        if (dist >= mFurthest) {
            dist = mFurthest;
        } else if (!override && dist < 10.f) {
            dist = 10.f;
        } else if (override && dist <= mNearest) {
            dist = mNearest;
            mIsNearest = true;
        }
        mCameraDistance = dist;

        if (override) {
            if (mVanity.enabled || mPreviewMode) {
                mPreviewCam.offset = mCameraDistance;
            } else if (!mFirstPersonView) {
                mMaxCameraDistance = mCameraDistance;
            }
        } else {
            mDistanceAdjusted = true;
        }
    }

    void Camera::setCameraDistance()
    {
        if (mDistanceAdjusted) {
            if (mVanity.enabled || mPreviewMode) {
                mCameraDistance = mPreviewCam.offset;
            } else if (!mFirstPersonView) {
                mCameraDistance = mMaxCameraDistance;
            }
        }
        mDistanceAdjusted = false;
    }

    void Camera::setAnimation(NpcAnimation *anim)
    {
        mAnimation = anim;

        processViewChange();
    }

    void Camera::processViewChange()
    {
        if(isFirstPerson())
        {
            mAnimation->setViewMode(NpcAnimation::VM_FirstPerson);
            mTrackingNode = mAnimation->getNode("Camera");
            if (!mTrackingNode)
                mTrackingNode = mAnimation->getNode("Head");
            mHeightScale = 1.f;
        }
        else
        {
            mAnimation->setViewMode(NpcAnimation::VM_Normal);
            SceneUtil::PositionAttitudeTransform* transform = mTrackingPtr.getRefData().getBaseNode();
            mTrackingNode = transform;
            if (transform)
                mHeightScale = transform->getScale().z();
            else
                mHeightScale = 1.f;
        }
        rotateCamera(getPitch(), getYaw(), false);
    }

    void Camera::getPosition(osg::Vec3f &focal, osg::Vec3f &camera)
    {
        focal = getFocalPoint();

        osg::Quat orient =  osg::Quat(getPitch(), osg::Vec3d(1,0,0)) * osg::Quat(getYaw(), osg::Vec3d(0,0,1));

        osg::Vec3d offset = orient * osg::Vec3d(0, -mCameraDistance, 0);
        camera = focal + offset;
    }

    void Camera::togglePlayerLooking(bool enable)
    {
        mFreeLook = enable;
    }

    bool Camera::isVanityOrPreviewModeEnabled()
    {
        return mPreviewMode || mVanity.enabled;
    }

    bool Camera::isNearest()
    {
        return mIsNearest;
    }
}
