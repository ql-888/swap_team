# Multiple D435i Observation Candidates Design

## Goal

Teach several manually approved pre-observation poses where the wrist D435i
can clearly see the drone's tag, then reuse those poses relative to the current
Gemini tag pose.

## Capture contract

The recorder never publishes robot commands. The operator manually positions
the arm and presses Space in the D435i preview. Each candidate contains ten
fresh samples of joint feedback, `base_link -> gemini_apriltag_0`,
`d435i_color_optical_frame -> apriltag_0`, and
`base_link -> flange_link`, plus one Gemini and one D435i image. A candidate is
accepted only while joint and tag translations remain stable.

## Runtime selection

For each taught candidate, derive its fixed `Tag -> flange -> TCP` transform.
At runtime, try exact taught candidates in recorded order. If none passes IK,
workspace, and collision validation, try small Tag-frame translation offsets
around each taught candidate. Candidate orientation is never changed and
`axial_angles_deg` remains `[0.0]`; the previous 90/180/270-degree rotations are
removed.

## Compatibility and scope

If no multi-candidate record exists, retain the existing single taught record.
This feature changes only the Gemini-to-D435i pre-observation selection. It does
not change the final D435i grasp orientation, descent, or gripper behavior.
