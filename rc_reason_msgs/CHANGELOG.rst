0.6.0 (2026-10-07)
------------------

* ComputeGrasps and DetectItems srv: add optional dimensioning
* Item msg: add types BOX and TEXTURED_BOX and box field for dimensioned BoxPick items
* Grasp and SuctionGrasp msg: add tcp_id
* Item and ItemModel msg: add MAIL_ITEMS type
* add TriggerDump srv
* add ImageEvent msg
* SilhouetteMatchDetectObject srv: add optional object_segmentation_model for SilhouetteMatchAI
* Grasp msg: add stroke_per_finger_approach_mm and stroke_per_finger_grasp_mm
* support BoxPick+Match and ItemPickAI:
  add TexturedBox msg, ItemModel types TEXTURED_BOX, BAG, CONSUMER_GOODS and SHEET_METAL,
  Item types TEXTURED_RECTANGLE, BAG, CONSUMER_GOODS and SHEET_METAL

0.5.0 (2025-07-30)
------------------

* add overexposed field to SetHandEyeCalibrationPose_Response

0.4.0 (2024-11-20)
------------------

* Grasp msg: add priority, gripper_id and collision_checked
* LoadCarrier msg: add height_open_side for 3-sided LC
* CadMatchDetectObject srv: add pose_prior_ids and data_acquisition_mode
* SilhouetteMatchDetectObject srv: add object_plane_detection

0.3.1 (2023-06-15)
------------------

0.3.0 (2022-02-08)
------------------

* refactor and update for v22.01

0.2.1 (2021-01-16)
------------------

* fix package dependencies for ROS buildfarm

0.2.0 (2021-01-14)
------------------

* Added new error values to hand-eye calibration service call definition

0.1.0 (2020-06-03)
------------------

* initial release
