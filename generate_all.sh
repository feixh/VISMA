#!/bin/sh
DATAROOT=/local/Data/VISMA_experiments
OUTPUT=/home/visionlab/Data/VISMA_depth_stride5

python scripts/prepare_data_for_SfMLearner.py --dataroot $DATAROOT/clutter1 --output-dir $OUTPUT/clutter1
python scripts/prepare_data_for_SfMLearner.py --dataroot $DATAROOT/clutter2 --output-dir $OUTPUT/clutter2
python scripts/prepare_data_for_SfMLearner.py --dataroot $DATAROOT/leather_chair --output-dir $OUTPUT/leather_chair
python scripts/prepare_data_for_SfMLearner.py --dataroot $DATAROOT/occlusion1 --output-dir $OUTPUT/occlusion1
python scripts/prepare_data_for_SfMLearner.py --dataroot $DATAROOT/occlusion2 --output-dir $OUTPUT/occlusion2
python scripts/prepare_data_for_SfMLearner.py --dataroot $DATAROOT/swivel_chair --output-dir $OUTPUT/swivel_chair
python scripts/prepare_data_for_SfMLearner.py --dataroot $DATAROOT/swivel_chair_lateral --output-dir $OUTPUT/swivel_chair_lateral
python scripts/prepare_data_for_SfMLearner.py --dataroot $DATAROOT/double_swivel_chairs_whiteboard --output-dir $OUTPUT/double_swivel_chairs_whiteboard
