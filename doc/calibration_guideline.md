1. for rack calibration, in "Dataset Panel".

2.1. first, set OperationName to rack_bottom_left. -> capture Reference 1 -> Go To Label Latest Reference 1

2.2. labelling web: GB200_Rack_Bottom_Left_Corner and GB200_Rack_Bottom_Right_Corner. save.

3.1. second, set OperationName to rack_top_left. -> capture Reference 1 -> Go To Label Latest Reference 1

3.2. labelling web: GB200_Rack_Top_Left_Corner and GB200_Rack_Top_Right_Corner. save.

4. Go To Workflow Config Center, in "Workflow Panel".

5. Load Template -> workflow_example_positioning_fitting.json (rename file if needed) 

    -> modify "movej_to_pose" in Freedrive Mode.

    -> *modify "upload_view" - "reference_name" if needed. 

    -> * modify "get_result_fitting" - "template_points" if needed.

    -> Save Workflow. 

6. In Workflow Panel, Refesh -> select json file -> Run Current Selected Workflow

    -> *if it has problems, use "ros2 run ur15_workflow run_workflow.py --config xxx.json" to check detailed problems

7.  "Draw GB200 Rack" to check the result.
