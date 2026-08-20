# Platform placement preparation

This directory contains the new platform-placement entry point. It does not
modify `scripts/eye_to_grasp/` or `scripts/eye_in _grasp/`.

## Complete grasp-to-place program

`run_grasp_place_complete.sh` combines the verified workflows from initial
camera/robot startup and drone grasp through final platform placement. It uses
the Gemini 336L `Standard52h13` ID 0 marker for platform XY and heading, fixes
the support height at 170 mm, and ignores measured marker Z.

```bash
cd /home/guoyi/gy_ws/piper_grasp_shen
export ROS_DOMAIN_ID=1
scripts/to_platform/run_grasp_place_complete.sh \
  I_CONFIRM_COMPLETE_PLATFORM_PLACEMENT \
  I_HAVE_CLEARED_THE_WORKSPACE \
  I_CONFIRM_DRONE_AND_TAG_FIXED \
  I_HAVE_CLEARED_THE_TRANSPORT_PATH
```

The program performs the following sequence:

1. Run the complete `eye_to_grasp/boot_and_grasp.sh` workflow.
2. Pause until `I_CONFIRM_HELD_DRONE_TRANSPORT` is typed.
3. Lock the platform marker, lift 100 mm, transport at 30 degrees to 200 mm
   above the support plane, and align to the verified 20-degree attitude.
4. Pause until `I_CONFIRM_PLATFORM_ABOVE_DESCENT` is typed.
5. Descend 100 mm + 10 mm + 10 mm, replanning from fresh feedback each time.
6. Pause until `I_CONFIRM_DRONE_SUPPORTED_RELEASE` is typed.
7. Open the gripper exactly 10 mm from its current feedback position.
8. Preserve TCP XY and rotation while retreating 120 mm along base `+Z`.

An incorrect confirmation or end-of-input stops the program. It also stops if
the post-grasp gripper feedback is outside 58–68 mm, the detector is missing,
feedback is stale, a report is stale, IK/collision validation fails, or an
executor fails. It never skips a failed stage. The collision model does not
contain the drone or platform contact geometry, so each checkpoint still
requires direct operator inspection and an immediately accessible emergency
stop.

## Recommended online pre-placement debug

The guarded debug flow plans and executes one stage at a time. The transport
stages continuously hold the existing gripper position. Descent, partial
release, and open-gripper retreat use separate plan files, executors, and exact
confirmation strings; no command automatically advances to the next stage.

Start the existing Gemini 336L camera and the platform-only detector, then run
the plan-only action:

```bash
export ROS_DOMAIN_ID=1
scripts/run_apriltag_gemini336l_standard52h13.sh
scripts/to_platform/run_platform_preplace_debug.sh plan
```

Inspect the generated report before each separately confirmed motion:

```bash
scripts/to_platform/run_platform_preplace_debug.sh lift \
  I_CONFIRM_PLATFORM_SAFE_LIFT_ONLY

scripts/to_platform/run_platform_preplace_debug.sh transport \
  I_CONFIRM_PLATFORM_TRANSPORT_POSITION_ONLY

scripts/to_platform/run_platform_preplace_debug.sh align \
  I_CONFIRM_PLATFORM_TILTED_ALIGN_ONLY
```

Stop and inspect the held drone after every command. `lift` raises the current
TCP by 100 mm, `transport` moves the locked drone centre to the platform XY at
the provisional object Z of 370 mm with a 30 degree transport tilt, and
`align` matches the platform-tag heading at the same locked object position
while retaining a 20 degree tilt about tag X. Live IK diagnosis showed that the
horizontal pose at this XY/Z reaches the joint-4 limit, while this tilted debug
pose has adequate joint margin. It is not the final horizontal pre-placement
pose. No command automatically advances to the next stage.

## Separately gated placement stages

The verified live run planned and executed a 100 mm descent followed by two
separately planned 10 mm descents. Each plan uses current joint feedback and
preserves the current TCP orientation and XY:

```bash
scripts/to_platform/run_platform_preplace_debug.sh plan-descend
scripts/to_platform/run_platform_preplace_debug.sh descend \
  I_CONFIRM_PLATFORM_DESCEND_100MM_ONLY

scripts/to_platform/run_platform_preplace_debug.sh plan-descend-10mm
scripts/to_platform/run_platform_preplace_debug.sh descend-10mm \
  I_CONFIRM_PLATFORM_DESCEND_10MM_ONLY
```

The partial-release and retreat scripts are intentionally standalone. Plan the
10 mm gripper opening from fresh gripper feedback, execute it only after the
drone is supported, then plan a 120 mm same-XY/same-orientation TCP retreat from
fresh joint feedback. Their execution confirmations are
`I_CONFIRM_PLATFORM_OPEN_GRIPPER_10MM_ONLY` and
`I_CONFIRM_PLATFORM_OPEN_RETREAT_120MM_ONLY` respectively. A 200 mm retreat was
tested offline during the live run and rejected as unreachable; 120 mm was
reachable and completed successfully.

These are real-motion debug tools, not an automatic placement routine. Before
each execution, inspect the newly generated report, clear the payload path,
keep the physical emergency stop available, and confirm the current object and
gripper state. The collision model does not include the drone or platform
contact geometry.

## Complete grasp-then-platform sequence

To run the original eye-to-grasp sequence first and then move the held drone
above the platform target, use the new orchestration wrapper:

```bash
export ROS_DOMAIN_ID=1
scripts/to_platform/run_grasp_then_platform.sh \
  I_CONFIRM_GRASP_THEN_PLATFORM \
  I_HAVE_CLEARED_THE_WORKSPACE \
  I_CONFIRM_DRONE_AND_TAG_FIXED \
  I_HAVE_CLEARED_THE_TRANSPORT_PATH
```

The wrapper calls the unchanged `scripts/eye_to_grasp/boot_and_grasp.sh`,
verifies gripper feedback after the D435i grasp/retreat, starts only the
Standard52h13 detector for the platform phase, and then calls
`run_platform_above.sh`. It stops at the above-platform pose; descent and
release are intentionally not part of this first integrated run.

The current entry point only moves a held drone to the planned pose above the
platform. It does not descend or open the gripper.

## Start vision

Keep the existing Gemini 336L camera running, then start the dual detector:

```bash
export ROS_DOMAIN_ID=1
scripts/run_gemini336l_dual_tag_live.sh
```

The detector must publish either
`/perception/gemini336l_live/standard52h13/detections` (the dual viewer) or
`/perception/gemini336l_transport/standard52h13/detections` (the transport
detector). The live viewer can be left running while the target is captured.

## Plan and execute the above pose

The platform tag center is configured as `0.170 m` above the desktop and the
held-drone target is `0.200 m` above the tag. The target `z` is therefore
`0.370 m`; the measured tag `z` is deliberately ignored. The tag's horizontal
X direction determines the target yaw. The planner then applies the same
`object_from_tcp` transform used during the drone grasp, so the drone's bottom
orientation remains tied to the gripper geometry.

```bash
scripts/to_platform/run_platform_above.sh \
  I_CONFIRM_PLATFORM_ABOVE_ONLY \
  I_HAVE_CLEARED_THE_TRANSPORT_PATH
```

This command requires a currently held drone and an enabled Piper control node.
It samples and stability-checks the target, runs IK and collision checks, then
commands the held-drone transport pose. It never descends and never releases.

If later measurement shows that the 20 cm clearance refers to the drone bottom
rather than its AprilTag/object origin, adjust
`--object-height-above-tag-m` after measuring that fixed offset.
