#!/usr/bin/python

from .namespace import NameSpace

import os

import getpath
import bvh
import json
import animation
import log
import numpy as np

from collections import OrderedDict

# Bone used for determining the pose scaling (root bone translation scale)
COMPARE_BONE = "upperleg02.L"

class Skeleton(NameSpace):
    """This namespace wraps call which work with skeleton, rig, poses and expressions."""

    def __init__(self,api):
        self.api = api
        NameSpace.__init__(self)
        self.trace()
        self.human = self.api.internals.getHuman()

    def getSkeleton(self):
        """Get the current skeleton, or None if no skeleton is assigned"""
        return self.human.getSkeleton()

    def getBaseSkeleton(self):
        """Get the internal default skeleton, which is independent on selected rig"""
        return self.human.getBaseSkeleton()

    def getPoseAsBoneDict(self):
        """Return a dict containing all bone rotations"""
        skeleton = self.getSkeleton()
        if skeleton is None:
            # No rig is set
            return None
        return None

    def getPoseAsBVH(self):
        """Return a BVH object describing the current pose"""
        skeleton = self.getSkeleton()
        if skeleton is None:
            # No rig is set
            return None
        b = BVH()
        b.fromSkeleton(skeleton)

        return b

    def getPoseAsAnimation(self):
        return self.human.getActiveAnimation()
 
    def clearPoseAndExpression(self):
        """Put skeleton back into rest pose"""
        human.resetToRestPose()
        human.removeAnimations()

    def setPoseFromFile(self, bvh_file_name):
        """Set the pose from a BVH file"""

        if bvh_file_name is None:
            # clear pose
            self.human.resetToRestPose()
            self.bvh_bone_length = None
            self.bvh_root_translation = None
            return
        
        if os.path.splitext(bvh_file_name)[1].lower() == '.bvh':
            anim = self._loadBvh(bvh_file_name, convertFromZUp="auto")
            if not anim:
                log.error('Cannot load animation from %s' % bvh_file_name)
                self.human.resetToRestPose()
                self.bvh_bone_length = None
                self.bvh_root_translation = None
                return
        else:
            log.error("Cannot load pose file %s: File type unknown." % bvh_file_name)
            return
        
        self.human.addAnimation(anim)
        self.human.setActiveAnimation(anim.name)
        self.human.setToFrame(0, update=False)
 
        self.human.setPosed(True)    
        pass

    def setExpressionFromFile(self, mhposeFile):
        """Set the expression from a mhpose file"""

        if mhposeFile is None:
            # clear expression

            original_pose = self.getPoseAsAnimation()
            if original_pose and hasattr(original_pose, 'pose_backref'):
                original_pose = original_pose.pose_backref
    
            if original_pose is None:
                self.human.setActiveAnimation(None)
            else:
                if self.human.hasAnimation(original_pose.name):
                    self.human.setActiveAnimation(original_pose.name)
                else:
                    self.human.addAnimation(original_pose)
                    self.human.setActiveAnimation(orgiginal_pose.name)
    
            if self.human.hasAnimation('expr-lib-pose'):
                self.human.removeAnimation('expr-lib-pose')
        else:
            # Assign expression
            
            base_bvh = bvh.load(getpath.getSysDataPath('poseunits/face-poseunits.bvh'), allowTranslation="none")
            base_anim = base_bvh.createAnimationTrack(self.human.getBaseSkeleton(), name="Expression-Face-PoseUnits")

            poseunit_json = json.load(open(getpath.getSysDataPath('poseunits/face-poseunits.json'), 'r', encoding='utf-8'), object_pairs_hook=OrderedDict)
            poseunit_names = poseunit_json['framemapping']

            base_anim = animation.PoseUnit(base_anim.name, base_anim._data, poseunit_names)

            face_bone_idxs = sorted(list(set([bIdx for l in base_anim.getAffectedBones() for bIdx in l])))

            new_pose = animation.poseFromUnitPose('expr-lib-pose', mhposeFile, base_anim)

            current_pose = self.getPoseAsAnimation()

            if current_pose is None:
                current_pose = new_pose
                current_pose.pose_backref = None
            else:
                if hasattr(current_pose,'pose_backref') and not current_pose.pose_backref is None:
                    current_pose = current_pose.pose_backref
                org_pose = current_pose
                current_pose = animation.mixPoses(org_pose, new_pose, face_bone_idxs)

            current_pose.name = 'expr-lib-pose'
            self.human.addAnimation(current_pose)
            self.human.setActiveAnimation(current_pose.name)
            self.human.setPosed(True)
            self.human.refreshPose()

    def _loadBvh(self, filepath, convertFromZUp="auto"):
        bvh_file = bvh.load(filepath, convertFromZUp)
        if COMPARE_BONE not in bvh_file.joints:
            msg = 'The pose file cannot be loaded. It uses a different rig then MakeHuman\'s default rig'
            #G.app.prompt('Error', msg, 'OK')
            log.error('Pose file %s does not use the default rig.' % filepath)
            return None
        anim = bvh_file.createAnimationTrack(self.human.getBaseSkeleton())
        if "root" in bvh_file.joints:
            posedata = anim.getAtFramePos(0, noBake=True)
            root_bone_idx = 0
            self.bvh_root_translation = posedata[root_bone_idx, :3, 3].copy()
        else:
            self.bvh_root_translation = np.asarray(3*[0.0], dtype=np.float32)
        self.bvh_bone_length = self._calculateBvhBoneLength(bvh_file)
        self._autoScaleAnim(anim)
        #_, _, _, license = self.getMetadata(filepath)
        #anim.license = license
        return anim
    
    def _createAnimationTrack(self, skeleton):
        pass

    def _calculateBvhBoneLength(self, bvh_file):
        import numpy.linalg as la
        if COMPARE_BONE not in bvh_file.joints:
            raise RuntimeError('Failed to auto scale BVH file %s, it does not contain a joint for "%s"' % (bvh_file.name, COMPARE_BONE))

        bvh_joint = bvh_file.joints[COMPARE_BONE]
        joint_length = la.norm(bvh_joint.children[0].position - bvh_joint.position)
        return joint_length
    
    def _autoScaleAnim(self, anim):
        """
        Auto scale BVH translations by comparing upper leg length to make the
        human stand on the ground plane, independent of body length.
        """
        import numpy.linalg as la
        bone = self.human.getBaseSkeleton().getBone(COMPARE_BONE)
        scale_factor = float(bone.length) / self.bvh_bone_length
        trans = scale_factor * self.bvh_root_translation
        log.message("Scaling animation %s with factor %s" % (anim.name, scale_factor))
        # It's possible to use anim.scale() as well, but by repeated scaling we accumulate error
        # It's easier to simply set the translation, as poses only have a translation on
        # root joint

        # Set pose root bone translation
        root_bone_idx = 0
        posedata = anim.getAtFramePos(0, noBake=True)
        posedata[root_bone_idx, :3, 3] = trans
        anim.resetBaked()
