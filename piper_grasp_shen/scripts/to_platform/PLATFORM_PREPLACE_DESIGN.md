# Platform pre-placement design

Date: 2026-08-20

## Scope

This stage moves an already grasped drone to a verified pose above the
platform. It stops before descent and never opens the gripper.

All new platform-placement code stays in `scripts/to_platform/`. The existing
drone grasp flow is not modified.

## Confirmed geometry and targets

- Platform marker: `Standard52h13`, ID `0`.
- Measured outer dashed-square edge: `36.5 mm`.
- apriltag_ros pose size: `21.9 mm` (`36.5 * 6 / 10`).
- The marker is horizontal and lies on the platform support plane.
- The support plane is provisionally `0.170 m` above the desktop/base plane.
- The marker centre is provisionally the desired drone centre in XY.
- The drone marker plane is parallel to the drone bottom.
- The desired drone heading is the same as the platform marker heading.
- The first pre-placement target puts the drone marker plane `0.200 m` above
  the support plane, so its fixed base-frame Z is `0.370 m`.

The `0.200 m` value is an initial online-debug target, not the later descent
distance. The operator will determine the descent only after inspecting the
pre-placement result.

## Vision policy

During the platform phase, only `Standard52h13` detections are consumed.
Before motion, collect multiple stable samples of the marker pose in the robot
base frame. Lock the averaged XY and marker heading. Ignore measured marker Z
when constructing the target.

Once locked, the target is not updated during transport. Occlusion by the
approaching drone therefore cannot move the target.

## Motion stages

1. **Capture and plan only**: lock the marker target, generate all stages, run
   workspace, IK, joint-limit, and collision checks, and publish no command.
2. **Safe lift**: raise the held drone using a separately executable stage,
   then stop for operator inspection.
3. **Transport position**: move to the platform target XY and provisional Z
   using a feasible transport orientation, then stop for inspection.
4. **Reachable debug alignment**: at the same XY/Z, match the platform-marker
   heading while retaining a 20 degree tilt about marker X, then stop.

Transport does not require the drone to remain horizontal. Live IK diagnosis
found that the horizontal, tag-aligned pose at object Z `0.370 m` drives joint
4 to its `+89 deg` limit. The operator therefore selected the reachable
20-degree tilted pose for this first debug run. A future descent/leveling stage
must be separately designed and confirmed before the drone is released.

The planner may try multiple IK seeds and intermediate waypoints. It must not
silently change final XY, fixed Z, selected 20-degree tilt, or heading.

## Execution gates

Each stage has its own explicit confirmation token. Planning never implies
permission to move. The first online-debug run ends after pre-placement
alignment; descent and gripper release are out of scope.

Before every real stage, require:

- current joint feedback and Piper control node;
- gripper feedback consistent with the held drone;
- a fresh, validated plan derived from the locked target;
- operator present, payload secure, path clear, platform stationary, and
  emergency stop ready.

## Failure behaviour

- Reject missing, stale, wrong-family, wrong-ID, or unstable detections.
- Reject marker tilt inconsistent with the horizontal support plane.
- Reject missing joint/gripper feedback.
- Reject IK, workspace, joint-limit, or collision-check failures.
- Never continue automatically to a later stage after a failure.
- Never descend or release in this version.

## Verification

Automated tests cover size conversion, fixed-Z construction, XY/yaw locking,
the selected 20-degree debug tilt, stage ordering, confirmation gates, and
refusal of descent/release. Before hardware motion, run the offline planner
from live feedback and inspect the generated target poses, joint solutions,
joint margins, and collision report.
